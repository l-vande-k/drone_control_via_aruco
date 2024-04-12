
import math
import os
import queue
import uuid
import threading
import numpy as np

import cv2
import cv2.aruco as aruco

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
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
        self.processing_thread.start()
        print("processing started")

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
        while self.running:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # this section adds the frames to a deque object so that we can access the frames independent of this thread when we need to
            yuv_frame_storage.append(yuv_frame)
            yuv_2d_array = yuv_frame.as_ndarray()
            yuv_frame_2dArray_storage.append(yuv_2d_array)
            if self.frame_count < 25:
                self.frame_count += 1
            else:
                yuv_frame_2dArray_storage.popleft()
                yuv_frame_storage.popleft()

            yuv_frame.unref()


    def frame_processing(self):
        
        frames_edited = 0

        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # the aruco ID must be below 50
        parameters = aruco.DetectorParameters()

        while True:

            if self.stop_processing == True:
                break
            
            if self.frame_count > 24:

                # variables for image processing
                k = np.array([[996.80114623, 0., 690.13286978],
                            [0., 961.38301889, 361.68868703],
                            [0., 0., 1.]])
                
                d = np.array([0.01849482, 0.01653952, -0.0063708, 0.0083632, -0.21739897])

                # this creates the flag that is passed into the following lines that is used for bgr conversion                
                cv2_cvt_color_flag = {
                olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
                }[yuv_frame_storage[-2].format()]

                # yuv --> bgr --> gray
                bgr_frame = cv2.cvtColor(yuv_frame_2dArray_storage[-2], cv2_cvt_color_flag)
                gray_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
                
                corners, ids, _ = cv2.aruco.detectMarkers(gray_frame, dictionary, parameters=parameters)
                
                if len(corners) > 0:
                    for i in range(0, len(ids)):
                        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.02, k, d, markerLength=50.0)
                        rot, _ = cv2.Rodrigues(rvec)

                        sy = math.sqrt(rot[0,0] * rot[0,0] +  rot[1,0] * rot[1,0])

                        singular = sy < 1e-6
                    
                        if  not singular :
                            x = math.atan2(rot[2,1] , rot[2,2])
                            y = math.atan2(-rot[2,0], sy)
                            z = math.atan2(rot[1,0], rot[0,0])
                        else :
                            x = math.atan2(-rot[1,2], rot[1,1])
                            y = math.atan2(-rot[2,0], sy)
                            z = 0
                    
                        eulers = np.array([x, y, z])
                        
                        
                        self.yaw = eulers[2]
                        self.x = tvec[0][0][0]
                        self.y = tvec[0][0][1]

                        #print("tvec: ", tvec)
                        #print("x&y: ", self.x, ", ", self.y)

                frames_edited += 1
    

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

    def fly(self):
        self.drone(
            FlyingStateChanged(state="hovering")
            | (TakeOff() & FlyingStateChanged(state="hovering"))
        ).wait()

        print("x&y: ", 2*self.x, ", ", 2*self.y)
        self.drone(moveBy(2*self.x, 2*self.y, 0, self.yaw, _timeout=20)).wait()

        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()


# variables used in threads
yuv_frame_2dArray_storage = deque()
yuv_frame_storage = deque()


def test_streaming():
    drone = StreamingExample()
    # Start the video stream
    drone.start()

    drone.move_gimbal(-90)
    # Perform some live video processing while the drone is flying
    drone.fly()

    drone.move_gimbal(0)
    # Stop the video stream
    drone.stop()

    print("done")


if __name__ == "__main__":
    test_streaming()