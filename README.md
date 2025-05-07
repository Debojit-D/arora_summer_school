# HEAL Challenge and Task Replication Suite

Welcome to the **HEAL Challenge Task Suite**!
This repository hosts a collection of engaging and technically enriching challenges designed to test manual and semi-autonomous operation of the **HEAL robotic arm**, inspired by real-world tasks, control paradigms, and interactive games.

---

## 📋 Task List

Each challenge includes:

* **Student Template**: A partially implemented script for completing the challenge.
* **Solution Code**: A reference implementation demonstrating successful task completion.
* **Documentation**: Setup, usage, and extension guidelines.

---

### 1. Relative Angle Reaching Task

**Description**:
In this task, the user must match a randomly generated **relative joint configuration** using a **joint-level teleoperation interface**. The robot evaluates the accuracy and scores the user's performance based on joint-wise alignment with the target.

📖 [Detailed Instructions](docs/task_1_relative_angle_reach.md)

<div align="center">

[![Watch the video](https://img.youtube.com/vi/QQjjRGbvd3g/hqdefault.jpg)](https://youtu.be/QQjjRGbvd3g)

</div>

---

### 2. Gravity Compensation Task

**Description**:
Implement a torque-based gravity compensation controller, allowing the robot to be moved freely by hand with minimal resistance from the motors. You will also compute and publish the end-effector pose using forward kinematics (either via KDL or manual DH-based method).

📖 [Detailed Instructions](docs/task_2_gravity_compensation.md)

<div align="center">

[![Watch the video](https://img.youtube.com/vi/ewGwxI-yWAo/hqdefault.jpg)](https://youtu.be/ewGwxI-yWAo)

</div>

---

### 3. Fun and Fair Trajectory Tracking Challenge

**Description**:
In this challenge, the HEAL robot must trace **smooth and repeatable end-effector trajectories** using **velocity-based Damped Least Squares (DLS) inverse kinematics**.

You will implement and test multiple dynamic paths such as:

* 🔵 **Circular path** — move the robot in a circular path.
* 🔶 **Sine wave motion** — trace oscillations across a flat surface (e.g., for cleaning or drawing)
* 🔁 **Straight-line sweep** — back-and-forth motion between two X-axis bounds

Your task is to:

1. Select and implement any **three or more** trajectories.
2. Dynamically update the robot's Cartesian **position and orientation**.
3. Log and plot the **3D trajectory** traced by the end-effector.

🛡️ **Safety Constraint**:
By default, angular velocity control is **disabled** in the IK solver to protect robot cable winding.
You may optionally **enable angular components** if the tasks demand so but ensure safe operation.

📦 **Starter Template**:
Begin by editing `03_trajectory_tracking_student_template.py`.

📖 [Detailed Instructions](docs/task_3_trajectory_tracking.md)

<div align="center">

[![Watch the video](https://img.youtube.com/vi/kliNHYT9Iak/hqdefault.jpg)](https://youtu.be/kliNHYT9Iak)

</div>

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
│   ├── 01_relative_angle_reach_student_template.py
│   ├── 02_gravity_compensation_student_template.py
│   ├── 03_trajectory_tracking_student_template.py
│   ├── 04_pick_and_place_student_template.py
│   └── dls_velocity_commander.py
│
├── solutions/
│   ├── 01_relative_angle_reach_solution.py
│   ├── 02_gravity_compensation_and_end_effector_publisher.py
│   ├── 03_trajectory_tracking_task_solution.py
│   ├── 04_pick_and_place_solution.py
│   ├── dls_velocity_commander.py
│   ├── orientation_maintenance_solution.py
│   └── orientation_maintenance_solutionv0.py
│
├── utils/
│   ├── joint_teleop_heal.py
│   └── TRAJECTORY_PLANNERS/
│       └── trajectory_planners.py
│
├── docs/
│   ├── task_1_relative_angle_reach.md
│   ├── task_2_gravity_compensation.md
│   ├── task_3_trajectory_tracking.md
│   └── task_4_pick_and_place.md
│
├── include/
│   └── arora_summer_school/
│
├── CMakeLists.txt
├── package.xml
├── README.md
└── LICENSE
```

---