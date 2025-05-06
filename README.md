
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
In this task, the robot performs a full **pick-and-place** operation using velocity control and gripper actuation.
You will:

* Record the Cartesian pose of the object to be picked.
* Use the previously implemented **gravity compensation mode** to move the robot manually and log the desired **end-effector pose** (position + orientation).
* Plan a sequence of Cartesian waypoints using this pose and use **inverse kinematics and trajectory planning** to move the robot.
* Execute grasping and releasing actions via ROS topics.

💡 *Hint*: Refer to your gravity compensation script where you published the end-effector pose using the robot’s forward kinematics. Use that code to extract the object pickup coordinates.

📖 [Detailed Instructions](docs/task_4_pick_and_place.md)

<div align="center">

[![Watch the video](https://img.youtube.com/vi/x2lWNBFbgVk/hqdefault.jpg)](https://youtu.be/x2lWNBFbgVk)

</div>

---

Let me know if you'd like the video preview to be smaller, or want icons or badges added for visual polish!


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

