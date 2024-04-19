
import math
import os
import queue
import uuid
import threading
import numpy as np
import time
import pickle
from array import *
import csv

from itertools import zip_longest

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
    yaw = 0
    x = 0
    y = 0
    z = 0
    
    unique_filename = ""
    noAruco = True

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)


        # this creates a folder with a unique name in a known directory location
        self.unique_filename = str(uuid.uuid4())
        self.recorded_video = "/home/levi/Documents/drone_testing/drone_vids/" + self.unique_filename
        os.mkdir(self.recorded_video)

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
        This is currently turned off... it stops some error messages
        '''
        # self.drone.streaming.set_output_files(
        #     video=os.path.join(self.recorded_video, "streaming.mp4"),
        #     # metadata=os.path.join(self.recorded_video, "streaming_metadata.json"),
        # )

        # Setup your callback functions to do some live video processing
        # I don't know what these do
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        
        # Start video streaming
        self.drone.streaming.start()
        # self.renderer = PdrawRenderer(pdraw=self.drone.streaming) # comment this to disable the renderer
        self.running = True
        
        self.frame_grabbing_thread.start()
        

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
        # This function will be called by Olympe for each decoded YUV frame.
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
            yuv_frame_cache.append(yuv_frame)
            
            # yuv frame converted to 2d array added to double ended queue
            yuv_2d_array = yuv_frame.as_ndarray()
            yuv_frame_2dArray_cache.append(yuv_2d_array)
            
            # conditionals for 
            if self.frame_count < 30:
                self.frame_count += 1
                if self.frame_count >= 28 and thread_not_started:
                    self.processing_thread.start()
                    thread_not_started = False
            else:
                # pop old frames off
                yuv_frame_2dArray_cache.popleft()
                yuv_frame_cache.popleft()
                
            yuv_frame.unref()


    def frame_processing(self):
        
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # the aruco ID must be below 50
        parameters = aruco.DetectorParameters()
        
        # this grabs the color flag index from memory in order to solve an error
        color_flag_index = None
        file_path = "/home/levi/Documents/drone_testing/drone_control_via_aruco/image_processing_variables/color_flag_index.pickle"
        with open(file_path, 'rb') as file:
            # Deserialize and retrieve the variable from the file
            color_flag_index = pickle.load(file)
        
                # variables for image processing
        k = np.array([[921.3151587, 0., 666.84963128],
                    [0., 921.88843465, 354.19572665],
                    [0., 0., 1.]])
        
        d = np.array([1.13291286e-02, 3.14439185e-01, -5.19291075e-03, -2.07237003e-04, -5.95381063e-01])

        while True:
            
            # this ends the thread with a bool
            if self.stop_processing == True:
                break
            
            if yuv_frame_cache is not None and len(yuv_frame_cache) > 5 and yuv_frame_2dArray_cache is not None and len(yuv_frame_2dArray_cache) > 5:
                
                # this creates the flag that is passed into the following lines that is used for bgr conversion
                
                cv2_cvt_color_flag = {
                olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
                }[color_flag_index]


                # yuv --> bgr --> gray
                bgr_frame = cv2.cvtColor(yuv_frame_2dArray_cache[-2], cv2_cvt_color_flag)
                gray_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
                
                corners, ids, _ = cv2.aruco.detectMarkers(gray_frame, dictionary, parameters=parameters)
                
                if len(corners) > 0:
                    self.noAruco = False
                    for i in range(0, len(ids)):
                        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.02, k, d)
                        
                        # getting yaw
                        rot, _ = cv2.Rodrigues(rvec)
                        sy = math.sqrt(rot[0,0] * rot[0,0] +  rot[1,0] * rot[1,0])
                        singular = sy < 1e-6
                        if  not singular :
                            self.yaw = math.atan2(rot[1,0], rot[0,0])
                        else :
                            self.yaw = 0
                        
                        # here we get the translational values from the translation vector, tvec
                        # these measurements aren't accurate, they need to be enlarged
                        scaler = 1.0/0.4275

                        tvec = tvec*scaler
                        
                        # cv2.drawFrameAxes(bgr_frame, k, d, rvec, tvec, 0.01) 
                        
                        self.y = tvec[0][0][0] # +x axis in frame is +y on drone
                        self.x = -1*tvec[0][0][1] # -y axis in frame is +x on drone
                        self.z = tvec[0][0][2] # +z axis in frame is +z on drone
                        
                        # ===== this section is the SME filter ======
                        MA_width = 200
                        yaw_deque.append(self.yaw)
                        if len(yaw_deque) <= MA_width:
                            continue
                        else:
                            yaw_deque.popleft()
                        self.yaw = np.average(yaw_deque)
                else:
                    self.noAruco = True


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
        self.drone(moveBy(0, 0, 0, np.deg2rad(370), _timeout=20)).wait()


    def correct_land(self):
        
        start_time = time.time()
        
        while True:
            if not self.noAruco:
                break
        time.sleep(3) # this is a patch to wait for the gimbal to finish rotating
        
        # initializing temp variables
        
        temp_yaw = 0            # in rad
        temp_x = 0
        temp_y = 0
        temp_z = 0
        
        # tolerancing for landing conditions
        
        yaw_tol = 3                 # in degrees
        x_y_upper_tol = 5  / 100      # in cm
        x_y_lower_tol = 7   / 1000    # in mm
        z_min = 0.56                  # in m
        
        # gains for movement control inputs
        
        K_xy_upper = 1
        K_xy_lower = 0.12
        
        K_yaw = 1
        K_yaw_lower = 0.5
        K_z = 1/5
        
        K_xy_i = 0.02
        
        x_error_integral = 0
        y_error_integral = 0
        
        offset = 5 / 100        # cm
        
        while True:
            
            self.move_gimbal(-90)
            
            # conditionals
            
            x = self.x - offset
            
            upper_x_y_cond = abs(x) > x_y_upper_tol or abs(self.y) > x_y_upper_tol
            
            yaw_cond = abs(np.rad2deg(self.yaw)) < yaw_tol
            x_cond = abs(x) < x_y_lower_tol
            y_cond = abs(self.y) < x_y_lower_tol
            z_cond = abs(self.z) < z_min
            
            # break to land?
            
            if yaw_cond and x_cond and y_cond and z_cond:
                break
            if self.noAruco:
                break
            
            
            # pose adjustments if we aren't landing
            
            
            if upper_x_y_cond:
                temp_x = K_xy_upper*x
                temp_y = K_xy_upper*self.y
                temp_z = 0
                temp_yaw = 0
                
                lim = 0.07
                
                if temp_x > -lim and temp_x < 0:
                    temp_x = -lim
                elif temp_x < lim and temp_x > 0:
                    temp_x = lim
                
                if temp_y > -lim and temp_y < 0:
                    temp_y = -lim
                elif temp_y < lim and temp_y > 0:
                    temp_y = lim
                                
            elif not yaw_cond:
                temp_yaw = K_yaw*self.yaw
                temp_x = 0
                temp_y = 0
                temp_z = 0
                            
            elif not z_cond:
                temp_z = K_z*self.z
                temp_x = 0
                temp_y = 0
                temp_yaw = 0
            
            elif not x_cond or not y_cond:
                # this section contains the PI control
                # x_error_integral += x
                y_error_integral += self.y
                temp_x = K_xy_lower*x #+ K_xy_i*x_error_integral
                temp_y = K_xy_lower*self.y + K_xy_i*y_error_integral
                temp_z = 0
                temp_yaw = K_yaw_lower*self.yaw
            
            self.drone(moveBy(temp_x, temp_y, temp_z, temp_yaw, _timeout=5)).wait()
        
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=10)).wait()


# variables used in threads
yuv_frame_2dArray_cache = deque()
yuv_frame_cache = deque()
yaw_deque = deque()

def loop_control():
    drone = StreamingExample()
    # Start the video stream
    drone.start()
    
    # Perform some live video processing while the drone is flying
    drone.takeoff_spin()
    
    drone.move_gimbal(-90)
    
    # this runs the P controlled landing method
    drone.correct_land()

    drone.move_gimbal(0)
    
    # Stop the video stream
    drone.stop()

if __name__ == "__main__":
    loop_control()
