# drone_control_via_aruco

The drone control program consists of multiple threads that
manage the following:
  1. Requesting and queuing camera frames from the drone
gimbal camera
  2. Image processing via the OpenCV library to calculate
pose estimation relative to an ArUco marker
  3. Sending high level movement commands to reposition
the drone as necessary

![{F8BADD99-458F-42B0-8361-85FDDAEC56AB}](https://github.com/user-attachments/assets/7216e2fa-d326-4a2d-89a4-c87c747ba89e)
