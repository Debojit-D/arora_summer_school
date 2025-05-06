# 🛠️ Pick and Place Task — Student Documentation

## 📄 Overview

This task involves implementing a **Pick-and-Place pipeline** for the **HEAL robotic arm** using:

* **Cartesian pose tracking** via **inverse kinematics**
* **Joint velocity-based trajectory execution**
* **Gripper commands** for pick and release using ROS topics

You will complete a partially filled Python script (`pick_and_place_student_template.py`) that takes the robot through a set of poses, performs grasping and releasing, and returns to the home pose.

[![Watch the video](https://img.youtube.com/vi/x2lWNBFbgVk/hqdefault.jpg)](https://youtu.be/x2lWNBFbgVk)


---

## 🎯 Objectives

By the end of this task, you should be able to:

✅ Define a series of Cartesian poses in 3D space
✅ Solve inverse kinematics using KDL
✅ Generate joint velocity trajectories using quintic interpolation
✅ Trigger grasp and release actions with ROS messages
✅ Execute a full pick-and-place routine

---

## 🧠 Background Concepts

### 1. **Inverse Kinematics (IK)**

The robot must compute joint angles that achieve a desired end-effector pose (position + orientation).
This is done using **PyKDL** with the `ChainIkSolverPos_LMA` solver.

### 2. **Joint Velocity Control**

Instead of commanding joint positions directly, this task uses **joint velocities**, which are generated from a trajectory interpolator (quintic).

### 3. **Action Interfaces**

The robot accepts:

* `/robotA/grasp_action/goal` → to **close** the gripper
* `/robotA/release_action/goal` → to **open** the gripper

These are used at the appropriate points in the trajectory.

---

## 📁 File Location

Your code lives at:

```bash
arora_summer_school/challenges/pick_and_place_student_template.py
```

You can run it after building the workspace using:

```bash
rosrun your_package pick_and_place_student_template.py
```

---

## 🧩 Sections to Complete

### ✏️ 1. Define Target Poses

Inside the constructor of the `VelocityCommander` class, you will find this block:

```python
self.poses = [
    # TODO: Fill this with the required pick and place poses.
]
```

Each entry in this list is a dictionary like:

```python
{
    'pos': kdl.Vector(x, y, z),
    'quat': kdl.Rotation.Quaternion(qx, qy, qz, qw),
    'action': 'open' or 'close' or ''
}
```

You need to define **at least 6–7 poses**:

| # | Pose Description            | Action  |
| - | --------------------------- | ------- |
| 1 | Home pose (above object)    | "open"  |
| 2 | Move down to grasp object   | ""      |
| 3 | Close gripper (pick object) | "close" |
| 4 | Lift up after grasp         | ""      |
| 5 | Move to placement position  | ""      |
| 6 | Lower and release           | "open"  |
| 7 | Return to home              | ""      |

Use real Cartesian coordinates that your robot can reach. Orientation should generally point the gripper down toward the object.

---

### ✏️ 2. Test Gripper Actions

The following utility functions are provided:

```python
publish_grasp_action_goal(force=100)     # Close gripper
publish_release_action_goal()            # Open gripper
```

These are automatically called based on the `action` field you define in the pose dictionary.

---

### ✏️ 3. Trajectory Execution

The script uses your current joint state to compute a quintic velocity trajectory to the IK-computed target. This is done using:

```python
TrajectoryPlanner.quintic_joint_trajectory(...)
```

You don't need to edit this — just make sure your poses are valid, and the joint state topic is working.

---

## 🧪 How to Test

### ✅ Prerequisites:

* Robot URDF loaded into parameter server (`robot_description`)
* `/joint_states` is being published
* Velocity controller is active (`/velocity_controller/command`)
* Gripper topics `/robotA/grasp_action/goal` and `/robotA/release_action/goal` are available

### 🚀 Run the script:

```bash
rosrun your_package pick_and_place_student_template.py
```

You should see:

* The robot moving through each of your poses
* Logs confirming velocity publishing
* Gripper actions being executed at the right times

---

## 🧰 Troubleshooting

| Problem                       | Cause                  | Fix                                                      |
| ----------------------------- | ---------------------- | -------------------------------------------------------- |
| `robot_description` not found | URDF not loaded        | Use `roslaunch` to load the URDF                         |
| IK solver fails               | Pose unreachable       | Check the coordinates and ensure the robot can reach it  |
| Gripper not responding        | Topic mismatch         | Check if `/robotA/grasp_action/goal` is being published  |
| Robot not moving              | No velocity controller | Make sure the controller is running in your control loop |

---

Note : Check the Utils Folder some helpful trajectory planning class files.

---
