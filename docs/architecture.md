
# architecture.md — System Architecture

## Control Architecture

User Command
↓
Language Interface
↓
Vision System
↓
Object Detection
↓
Pose Estimation
↓
Motion Planner
↓
Robot Controller
↓
myCobot Arm

---

## Compute Layout

Laptop

Runs:

• perception
• policy inference
• planning
• simulation

Robot Pi

Runs:

• motor control
• gripper control
• serial communication

---

## Coordinate Frames

camera_frame  
robot_base_frame  
tool_frame  
object_frame  

---

## Calibration Workflow

1. Camera intrinsics calibration using **ChArUco board**
2. Camera extrinsics calibration using **AprilTag board on table**
3. Compute transform:

camera_frame → robot_base_frame

This enables:

pixel → world coordinate conversion

---

## Logging Data Schema

Each record should contain:

timestamp  
joint_command[6]  
joint_state[6]  
gripper_state  
camera_frame_id  
detected_object_pose  

Logs stored as:

CSV or rosbag

Used for:

• latency measurement  
• real→sim parameter estimation
