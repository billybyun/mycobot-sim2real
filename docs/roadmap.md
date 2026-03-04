
# roadmap.md — Development Roadmap

## Phase 0 — Before Robot Arrives

Install:

ROS2 Humble  
Isaac Sim  
PyTorch  

Prepare:

AprilTag board  
ChArUco board  

Write:

camera calibration scripts  
AprilTag pose estimation  
logging infrastructure  
Isaac Sim scene stub

---

## Phase 1 — Hardware Validation

Verify robot control

Tests:

• move arm  
• open gripper  
• close gripper  

Example:

mc.send_angles([0,0,0,0,0,0],30)

---

## Phase 2 — Scripted Pick and Place

Hard-code object location

Sequence:

1 move above object  
2 lower arm  
3 close gripper  
4 lift  
5 move to drop zone  
6 open gripper

---

## Phase 3 — Add Camera

Mount overhead camera

Capture frames using OpenCV

---

## Phase 4 — Object Localization

Implement pixel → world conversion

Methods:

homography  
AprilTags

---

## Phase 5 — Vision Pick and Place

Robot picks object detected by vision

---

## Phase 6 — Language Layer

Example command:

"Pick the red cube"

Language module outputs task parameters

---

## Phase 7 — Simulation Integration

Import URDF into Isaac Sim

Validate:

• link geometry  
• joint limits  
• end-effector frame  

---

## Phase 8 — Sim → Real Transfer

Train policies in simulation

Domain randomization parameters:

mass ±20%  
friction ±50%  
joint damping ±30%  
action delay 50–150 ms  
camera noise Gaussian

---

## Phase 9 — Real → Sim Refinement

Log real robot data

Estimate:

• latency  
• friction  
• damping  

Update simulation parameters
