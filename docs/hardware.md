
# hardware.md — Hardware Setup

## Robot

Elephant Robotics **myCobot 280 Pi**

Specifications:

• 6 DOF  
• reach: 280 mm  
• payload: ~250 g  

Python control example:

from pymycobot import MyCobot, PI_PORT, PI_BAUD

mc = MyCobot(PI_PORT, PI_BAUD)

mc.power_on()
mc.send_angles([0,-60,90,0,45,0],20)

---

## Gripper

Adaptive electric gripper

Example:

mc.set_gripper_value(100,50)  # close
mc.set_gripper_value(0,50)    # open

---

## Camera

Initial configuration:

Overhead USB webcam

Advantages:

• easier calibration  
• minimal occlusion  

Optional upgrades:

Luxonis OAK-D Lite  
Intel RealSense

---

## Calibration Targets

AprilTag board

Uses:

• pose estimation
• coordinate alignment

ChArUco board

Used for:

camera intrinsic calibration
