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
Use a torque-based gravity compensation controller, allowing the robot to be moved freely by hand with minimal resistance from the motors. You will also compute and publish the end-effector pose using forward kinematics (either via KDL or manual DH-based method).

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

**🛠️ Description**:
In this challenge, a laser pointer is mounted on the robot’s end-effector to localize a distant target using only internal sensing. The robot must infer the Cartesian position of the target based solely on:

* Joint encoder feedback
* Forward kinematics of the manipulator

This task simulates long-range target localization in environments where visual sensors are unavailable.

**🎯 Objective**:
Accurately determine the 3D Cartesian coordinates of the point where the laser intersects a surface or object, using internal robot state information alone.

---

**✅ Deliverables**:

* A working script or ROS node that:

  * Computes the laser ray from the end-effector frame using joint state feedback.
  * Determines the laser's intersection point in the world frame (target localization).
  * Logs or outputs the inferred Cartesian coordinates.

📖 [Detailed Instructions](docs/task_5_navy_challenge.md)

---
### Running the Robot and Scripts

Note : Make sure you run this in the normal system environment. If Conda is active, your terminal prompt will show (base). Please deactivate Conda before running any robot scripts, below.

Follow these steps to start the robot and run the teleoperation scripts:

#### Step 1: Start the Robot and Controller
1. **Power on the robot and its controller**.

2. **Connect to the robot's controller**:
   Use SSH to connect to the robot’s controller from your local PC:

   ```bash
   ssh cobot@192.168.1.25
   ```

   Enter the password when prompted. This establishes a connection with the robot's controller.

3. **Navigate to the controller’s execution directory**:

   ```bash
   cd tests/build/
   ```

   This will take you to the required directory to run the robot's server.

4. **Start the Gripper**:

   To start the **gripper** and initiate communication with the hardware, use the following command:

   ```bash
   sudo ./gripper_test
   ```

    Once it’s running, the gripper should respond to commands and be ready for use in the teleoperation setup. Keep running `/gripper_test` till the gripper on the heal robot opens and closes. If you get an error run the ' /gripper_test' again.
5. **Start the Heal server**:

   ```bash
   sudo ./heal_server
   ```

   This launches the server that manages the robot's hardware and communication. Do not close this terminal.


Note: If the system is already in the home position, you can skip the step below and move on to the next section.

6. **Home the robot**:
   To bring the robot to its home position, run the following command: Note: This command should be executed on the system you’ve SSHed into (i.e., the remote system).
These are individual command — even if the server is already running, you should first terminate it by pressing Ctrl+C. Then, run the commands below. 



   ```bash
   sudo ./base_rigid
   ```

   Make sure to run this command before shutting down the robot to ensure it homes correctly. Once done press `ctrl+c`.


---

#### Step 2: Launch the Robot Controller
1. **Open a new terminal on your PC** and navigate to the directory where the controller is stored:

   ```bash
   cd Debojit_WS/Addverb_Heal_and_Syncro_Hardware
   ```

2. **Source the workspace** to set up the environment variables:

   ```bash
   source devel/setup.bash
   ```
---

### 3. **Launch the Controller using ROS**

To bring up the robot control system, run:

```bash
roslaunch addverb_cobot_control bringup.launch
```

If you see the message:

```
[INFO] [timestamp]: robot has started
```

…then the robot is successfully initialized, and you can proceed to run your control scripts.

---

### 4. **Check the Control Mode**

Open the configuration file:

```bash
src/cobot_ros/addverb_cobot_control/config/default_control.yaml
```

Ensure the following line is set to **`velocity`** control mode:

```yaml
ros_control_mode: velocity
```

> 🔄 **If it is set to `effort`**, change it to `velocity`.
> Use `effort` **only** for:
>
> * Gravity compensation tasks
> * Recording Cartesian poses passively

---

### 5. **Modify the Launch File to Include Controller Spawner**

In your `bringup.launch` file (or included launch files), add the following node block **if not already present**:

Path :

```bash
src/cobot_ros/addverb_cobot_control/launch/bringup.launch
```

```xml
<!-- Load controller manager -->
<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
      args="joint_state_controller effort_controller twist_controller joint_trajectory_controller" />
```

> 🔧 **Note**: Replace `velocity_controller` with `effort_controller` as requested, which is appropriate when operating under effort-based control mode (e.g., for gravity compensation).

---

### ⚠️  Shutting Down and Caution

🔴 Always sit with the emergency stop button while operating the robot. (VERY IMPORTANT)

If you notice any undesired or unsafe behavior, immediately press the emergency stop.

Note : For restarting the system after emergency stop is released follow from the starting of Running the Robot and Scripts

 🔻 Shutting Down Procedure:
1. First, shut down the robot via ROS using the following command:

   (In a new terminal)
   
   ```bash
   rosservice call /robotA/shutdown_robot_srv "data: true"
   ```
2. Then, return to the terminal where you ran the heal server command
   and press Ctrl+C to terminate it.

 ✅ Always follow this procedure when the robot is not in use.


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
│   ├── 05_navy_challenge_student_template.py
│   └── dls_velocity_commander.py
│
├── solutions/
│   ├── 01_relative_angle_reach_solution.py
│   ├── 02_gravity_compensation_and_end_effector_publisher.py
│   ├── 03_trajectory_tracking_task_solution.py
│   ├── 04_pick_and_place_solution.py
│   ├── 05_navy_challenge_solution.py
│   └── dls_velocity_commander.py
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
│   ├── task_4_pick_and_place.md
|   └── task_5_navy_challenge.md
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
