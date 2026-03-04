
# myCobot Robotics Project Documentation (MASTER)

This is the **master documentation file** for the myCobot Sim2Real / VLA robotics project.
It serves as the entry point for humans and coding agents (Cursor, Claude, Copilot).

---

## Documentation Structure

### Project Plan
High-level goals and sim↔real research loop.

→ [plan.md](plan.md)

### System Architecture
Full system architecture, calibration workflow, and logging schema.

→ [architecture.md](architecture.md)

### Hardware Setup
Robot, gripper, camera, and calibration targets.

→ [hardware.md](hardware.md)

### Development Roadmap
Step-by-step build plan.

→ [roadmap.md](roadmap.md)

### Repository Structure
Python module layout designed for coding agents.

→ [repo_structure.md](repo_structure.md)

---

## System Overview

User Command
↓
Language Parser
↓
Vision System
↓
Object Localization
↓
Motion Planner
↓
Robot Controller
↓
myCobot Arm

Goal demo:

"Pick up the red object and place it in the bin."
