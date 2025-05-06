
# HEAL Challenge and Task Replication Suite

Welcome to the **HEAL Challenge Task Suite**!
This repository hosts a collection of engaging and technically enriching challenges designed to test manual and semi-autonomous operation of the **HEAL robotic arm**, inspired by real-world tasks, control paradigms, and interactive games.

---

## 📋 Task List

Each challenge includes:

* **Challenge Code**: Presents the task interface or control.
* **Solution Code**: A reference implementation demonstrating successful task completion.
* **Documentation**: Setup, usage, and extension guidelines.

---

### 1. Relative Angle Reaching Task

**Description**:
The user is given a random target joint configuration relative to the current pose. Using a joint-level teleoperation interface, the user must drive the robot to this configuration. The system scores performance based on joint-wise accuracy.

---

### 2. Gravity Compensation Task

**Description**:
Implement a torque-based gravity compensation controller, allowing the robot to be moved freely by hand with minimal resistance from the motors.

---

### 3. Fun and Fair Trajectory Tracking Challenge

**Description**:
The robot must trace a complex closed-loop trajectory manually or semi-autonomously. A buzzer integrated into the system activates when the robot is accurately following the path.


---

### 4. Pick and Place Task

**Description**:
Record the cartesian position of where the object needs to be picked using, Hint : In the gravity compensation you had worked on creating a publisher for computing the end-effector pose and orientation usign that get the predefined coordinated where you need to pick the object and then perform a series of planning to place the object to a new location

---

### 5. Navy Challenge – Target Localization via Laser

**Description**:
A laser mounted on the end-effector is used to localize a distant target. Using internal joint sensing and minimal visual feedback, the robot must identify the Cartesian location of the target point.

**Deliverables**:

* Target detection and localization script
* Sensor data processing module

---

## 📦 Repository Structure

```plaintext
arora_summer_school/
│
├── challenges/
│   ├── relative_angle_reach.py
│   ├── gravity_compensation.py
│   ├── trajectory_tracking.py
│   ├── pick_and_place.py
│   ├── navy_target_localization.py
│
├── solutions/
│   ├── relative_angle_reach_solution.py
│   ├── gravity_compensation_solution.py
│   ├── trajectory_tracking_solution.py
│   ├── pick_and_place_solution.py
│   ├── navy_target_localization_solution.py
│
├── docs/
│   ├── setup_instructions.md
│   ├── hardware_requirements.md
│   ├── software_installation.md
│   └── task_specific_notes/
│       ├── relative_angle_reach.md
│       ├── gravity_compensation.md
│       ├── trajectory_tracking.md
│       ├── pick_and_place.md
│       ├── navy_target_localization.md
│
├── README.md
└── LICENSE
```

---

