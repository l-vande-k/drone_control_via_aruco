
import math
import os
import queue
import uuid
import threading
import numpy as np
import time
import pickle

import cv2
import cv2.aruco as aruco

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
import olympe.messages.gimbal as gimbal
from olympe.video.renderer import PdrawRenderer

from collections import *


olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")


class StreamingExample:

    frame_count = 0
    stop_processing = False
    yaw = -math.pi
    x = 0
    y = 0
    z = 0

    noAruco = False

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)


        # this creates a folder with a unique name in a known directory location
        unique_filename = str(uuid.uuid4())
        self.recorded_video = "/home/levi/Documents/drone_testing/drone_vids/" + unique_filename
        os.mkdir(self.recorded_video)
        #print(f"Olympe streaming example output dir: {self.recorded_video}")


        self.frame_queue = queue.Queue()
        self.frame_grabbing_thread = threading.Thread(target=self.yuv_frame_grabbing)
        self.processing_thread = threading.Thread(target=self.frame_processing)

        self.renderer = None

    def start(self):
        # Connect to drone
        assert self.drone.connect(retry=3)

        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"

        '''
        You can record the video stream from the drone if you plan to do some post processing.
        This is currently turned off... it stops some errors
        '''
        # self.drone.streaming.set_output_files(
        #     video=os.path.join(self.recorded_video, "streaming.mp4"),
        #     metadata=os.path.join(self.recorded_video, "streaming_metadata.json"),
        # )

        # Setup your callback functions to do some live video processing
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
        self.drone.streaming.start()
        self.renderer = PdrawRenderer(pdraw=self.drone.streaming)
        self.running = True
        
        self.frame_grabbing_thread.start()
        print("frame grabbing started")
        

    def stop(self):
        self.stop_processing = True
        self.running = False
        self.frame_grabbing_thread.join()
        if self.renderer is not None:
            self.renderer.stop()

        # Properly stop the video stream and disconnect
        assert self.drone.streaming.stop()
        assert self.drone.disconnect()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.

            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)



    # this function grabs the frames and drops them into a deque object for the processing thread
    def yuv_frame_grabbing(self):
        thread_not_started = True
        while self.running:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # this section adds the frames to a deque object so that we can access the frames independent of this thread when we need to
            
            # unatlered yuv frame with all info attached added to double ended queue
            yuv_frame_storage.append(yuv_frame)
            
            # yuv frame converted to 2d array added to double ended queue
            yuv_2d_array = yuv_frame.as_ndarray()
            yuv_frame_2dArray_storage.append(yuv_2d_array)
            
            # conditionals for 
            if self.frame_count < 30:
                self.frame_count += 1
                if self.frame_count >= 28 and thread_not_started:
                    self.processing_thread.start()
                    print("processing started")
                    thread_not_started = False
            else:
                # pop old frames off
                yuv_frame_2dArray_storage.popleft()
                yuv_frame_storage.popleft()
                
            yuv_frame.unref()


    def frame_processing(self):
        
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # the aruco ID must be below 50
        parameters = aruco.DetectorParameters()
        
        color_flag_index = None
        file_path = "/home/levi/Documents/drone_testing/drone_control_via_aruco/image_processing_variables/color_flag_index.pickle"
        with open(file_path, 'rb') as file:
            # Deserialize and retrieve the variable from the file
            color_flag_index = pickle.load(file)
        
        # thes values are for validating that there has been no aruco sighting

        while True:

            if self.stop_processing == True:
                print("told to stop processing")
                break
            
            if yuv_frame_storage is not None:

                # variables for image processing
                k = np.array([[996.80114623, 0., 690.13286978],
                            [0., 961.38301889, 361.68868703],
                            [0., 0., 1.]])
                
                d = np.array([0.01849482, 0.01653952, -0.0063708, 0.0083632, -0.21739897])
                                
                # this creates the flag that is passed into the following lines that is used for bgr conversion                
                cv2_cvt_color_flag = {
                olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
                }[color_flag_index]

                # yuv --> bgr --> gray
                bgr_frame = cv2.cvtColor(yuv_frame_2dArray_storage[-2], cv2_cvt_color_flag)
                gray_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
                
                corners, ids, _ = cv2.aruco.detectMarkers(gray_frame, dictionary, parameters=parameters)
                
                if len(corners) > 0:
                    for i in range(0, len(ids)):
                        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.02, k, d)
                        rot, _ = cv2.Rodrigues(rvec)

                        # getting yaw
                        rot, _ = cv2.Rodrigues(rvec)
                        sy = math.sqrt(rot[0,0] * rot[0,0] +  rot[1,0] * rot[1,0])
                        singular = sy < 1e-6
                        if  not singular :
                            self.yaw = math.atan2(rot[1,0], rot[0,0])
                        else :
                            self.yaw = 0
                        
                        # these measurements aren't accurate, they need to be enlarged
                        gain = 2
                        
                        self.y = gain*tvec[0][0][0] # +x axis in frame is +y on drone
                        self.x = -1*gain*tvec[0][0][1] # -y axis in frame is +x on drone
                        self.z = tvec[0][0][2] # +z axis in frame is +z on drone
                # else:
                #     self.noAruco = True
    

    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        while not self.frame_queue.empty():
            self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def move_gimbal(self,attitude):
        self.drone(
            gimbal.set_target(
                gimbal_id = 0,
                control_mode = "position",
                yaw_frame_of_reference = "absolute",
                yaw = 0.0,
                pitch_frame_of_reference = "absolute",
                pitch = attitude,
                roll_frame_of_reference = "absolute",
                roll = 0.0
            )
        ).wait()


    def takeoff_spin(self):
        self.drone(
            FlyingStateChanged(state="hovering")
            | (TakeOff() & FlyingStateChanged(state="hovering"))
        ).wait()
        
        # take a 360 pan of the room for post processing
        self.drone(moveBy(0, 0, 0, np.deg2rad(-30), _timeout=20)).wait()

    def correct_land(self):
        
        
        while True:
            if not self.noAruco:
                break
            
        # initializing temp variables
        
        temp_yaw = 0            # in rad
        temp_x = 0
        temp_y = 0
        temp_z = 0
        
        # tolerancing for landing conditions
        
        yaw_tol = 5             # in degrees
        x_y_tol_upper = 1       # in m
        x_y_tol = 50 / 1000     # in mm
        z_min = 0.75            # in m
        
        # gains for movement control inputs
        
        K_xy_upper = 1
        K_xy = 0.2
        
        K_yaw = 1
        K_z = 1/3
        K_gaz = 0.5
        
        K_xy_i = 0.1
        
        x_error_integral = 0
        y_error_integral = 0
        
        while True:
            
            yaw_mode = False
            xyz_mode = False
            
            # print("no aruco?  ", self.noAruco)
                        
            yaw_cond = abs(np.rad2deg(self.yaw)) < yaw_tol
            x_cond_upper = abs(self.x) < x_y_tol_upper
            y_cond_upper = abs(self.y) < x_y_tol_upper
            x_cond = abs(self.x) < x_y_tol
            y_cond = abs(self.y) < x_y_tol
            z_cond = abs(self.z) < z_min
            
            # break to land?
            
            if yaw_cond and x_cond and y_cond and z_cond:
                print("all landing criteria met")
                break
            if self.noAruco:
                print("no aruco found")
                break
            
            # correctional conditions, we don't want to over correct or worry about things that are close enough
            
            if not yaw_cond:
                temp_yaw = self.yaw
                temp_x = 0
                temp_y = 0
                temp_z = 0
                yaw_mode = True
                print("correcting yaw")
            
            elif not x_cond_upper or not y_cond_upper:
                if self.x < 0:
                    temp_x = -1
                elif self.x > 0:
                    temp_x = 1
                
                if self.y < 0:
                    temp_y = -1
                elif self.y > 0:
                    temp_y = 1
                    
                temp_z = 0
                temp_yaw = 0
                
                xyz_mode = True
                print("correcting large x&y")
            
            elif not x_cond or not y_cond:
                temp_x = self.x
                temp_y = self.y
                temp_yaw = 0
                temp_z = 0
                xyz_mode = True
                print("correcting small x&y")
                
            # elif not z_cond:
            #     temp_z = K_z*self.z
            #     temp_x = 0
            #     temp_y = 0
            #     temp_yaw = 0
            #     print("correcting z")
                        
            r_p_max = 3
            yaw_max = 15
            gaz_max = 100
            
            flag = 1
            roll = round(    r_p_max*(temp_y)  )
            pitch = round(   r_p_max*(temp_x)  )
            yaw = round(     yaw_max*(K_yaw*(temp_yaw))  )
            gaz = round(    -gaz_max*K_gaz*temp_z  )
            timestampAndSeqNum = 0
            
            print(self.yaw)
            print("yaw input: ", yaw)
            print("roll: ", roll)
            print("pitch: ", pitch)
            print("gaz: ", gaz)
            print("\n")
            
            if yaw_mode:
                self.drone(moveBy(0, 0, 0, temp_yaw, _timeout=5)).wait()
            if xyz_mode:
                self.drone(PCMD(flag, roll, pitch, 0, gaz, timestampAndSeqNum, _timeout=2))#.wait()
        
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()
        print("x: ", self.x, ", y: ", self.y, ", z: " , self.z, ", yaw: ", np.rad2deg(self.yaw), "\n")



# variables used in threads
yuv_frame_2dArray_storage = deque()
yuv_frame_storage = deque()


def loop_control():
    drone = StreamingExample()
    # Start the video stream
    drone.start()
    
    # Perform some live video processing while the drone is flying
    drone.takeoff_spin()

    drone.move_gimbal(-90)

    print("landing sequence started")
    
    drone.correct_land()

    drone.move_gimbal(0)
    # Stop the video stream
    drone.stop()

    print("done")


if __name__ == "__main__":
    loop_control()