# HEAL Challenge and Task Replication Suite

Welcome to the **HEAL Challenge Task Suite**!  
This repository contains a set of fun and technically interesting challenges designed to test manual and semi-autonomous operation of the HEAL robotic arm, inspired by real-world tasks and games.

## 📋 Task List

Each challenge consists of:
- **Challenge Code**: Presents the task to the user.
- **Solution Code**: Reference solution demonstrating the correct behavior.
- **Documentation**: Setup, execution, and extension instructions.

---

### 1. Relative Angle Reaching Task

**Description**:  
Random relative joint angles are provided to the user. The user must manually move the robot to match the target configuration. The system scores the performance based on accuracy.

**Deliverables**:
- Challenge script
- Scoring system

---

### 2. Orientation Maintenance Challenge ("Shooting Shahrukh Khan")

**Description**:  
The robot must maintain the end-effector orientation pointing towards a target (e.g., Shahrukh Khan), even when one or two intermediate joints are manually perturbed.

**Deliverables**:
- Challenge script
- Orientation maintenance controller
- Manual override interface

---

### 3. Fun and Fair Trajectory Tracking Challenge

**Description**:  
The robot must follow a complex closed-loop trajectory manually or semi-autonomously. Buzzer integration on the loop will provide feedback when the trajectory is correctly tracked.

**Deliverables**:
- Trajectory visualization and tracking script
- Buzzer integration module

---

### 4. Navy Challenge – Target Localization via Laser

**Description**:  
Using a laser mounted at the end-effector, the robot must determine the Cartesian location of a target based on internal sensing and minimal external inputs.

**Deliverables**:
- Target detection and localization script
- Sensor data processing

---

### 5. Carrom Playing Task Replication

**Description**:  
Replicating the dynamics of a carrom striker using the robot arm: applying quick, planar impulses.

**Deliverables**:
- Striker shot trajectory planner
- Impact dynamics mimicry

---

### 6. Gravity Compensation Task

**Description**:  
Implement basic gravity compensation using torque control to allow the arm to be manually moved with minimal resistance.

**Deliverables**:
- Gravity compensation controller

---

### 7. Force Production Task

**Description**:  
The robot must press against a static button setup, generating a controlled static force.

**Deliverables**:
- Force controller script
- Static force monitoring

---

## 📦 Repository Structure

```plaintext
arora_summer_school/
│
├── challenges/
│   ├── relative_angle_reach.py
│   ├── orientation_maintenance.py
│   ├── trajectory_tracking.py
│   ├── navy_target_localization.py
│   ├── carrom_playing_task.py
│   ├── gravity_compensation.py
│   ├── force_production_task.py
│
├── solutions/
│   ├── relative_angle_reach_solution.py
│   ├── orientation_maintenance_solution.py
│   ├── trajectory_tracking_solution.py
│   ├── navy_target_localization_solution.py
│   ├── carrom_playing_task_solution.py
│   ├── gravity_compensation_solution.py
│   ├── force_production_task_solution.py
│
├── docs/
│   ├── setup_instructions.md
│   ├── hardware_requirements.md
│   ├── software_installation.md
│   ├── task_specific_notes/
│       ├── relative_angle_reach.md
│       ├── orientation_maintenance.md
│       ├── trajectory_tracking.md
│       ├── navy_target_localization.md
│       ├── carrom_playing_task.md
│       ├── gravity_compensation.md
│       ├── force_production_task.md
│
├── README.md
└── LICENSE
