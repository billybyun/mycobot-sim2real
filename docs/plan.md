
# plan.md — myCobot Sim2Real / VLA Project

## Project Goal

Build a robotics research project using:

- myCobot 280 Pi robot arm
- vision-based manipulation
- simulation training
- optional language interface

Key demonstrations:

• Vision-based manipulation  
• Sim → Real transfer  
• Real → Sim system identification  

---

## Sim ↔ Real Loop

REAL ROBOT
↓
Collect logs
↓
Fit simulation parameters
↓
Update simulation model
↓
Train policies in simulation
↓
Deploy to real robot
↓
Measure gap
↓
Repeat

This loop enables:

• Sim → Real policy transfer  
• Real → Sim system identification  
• Digital twin refinement

---

## Software Stack

OS  
Ubuntu 22.04

Simulation  
Isaac Sim

Vision  
OpenCV  
AprilTag  
ChArUco

Learning  
PyTorch

Control  
pymycobot Python API

---

## Core Repository Layout

sim2real/
├── sim/
├── ros2_ws/
├── vision/
├── policies/
├── data/
└── scripts/
