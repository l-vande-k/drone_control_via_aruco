import olympe
import os
import numpy as np

from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

from olympe.messages.skyctrl.CoPiloting import setPilotingSource

import time

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})


DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")

drone = olympe.Drone(DRONE_IP)

drone.connect()


drone(
    FlyingStateChanged(state="hovering")
    | (TakeOff() & FlyingStateChanged(state="hovering"))
).wait()

# drone.start_piloting()

flag = 1
roll = 0
pitch = 0
yaw = 0
gaz = -100
timestampAndSeqNum = 0

for i in range(3):
    drone(PCMD(flag, roll, pitch, yaw, gaz, timestampAndSeqNum, _timeout=10)).wait()
    print("sent!")

time.sleep(2)


# olympe.Drone.streaming.start()
drone(Landing()).wait()

# drone.stop_piloting()


drone(FlyingStateChanged(state="landed")).wait()
drone.disconnect()
