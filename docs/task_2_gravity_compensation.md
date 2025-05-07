# 🪂 Gravity Compensation + Forward Kinematics

## 📄 Overview

In this task, you will operate the robot in **gravity compensation (free-drive)** mode and compute the **forward kinematics (FK)** to estimate and visualize the robot's **end-effector pose** in real time.

You will write a ROS node that listens to joint states, computes the pose of the end-effector using one of two approaches, and publishes the result to a ROS topic for logging or visualization in RViz.

<div align="center">

[![Watch the video](https://img.youtube.com/vi/ewGwxI-yWAo/hqdefault.jpg)](https://youtu.be/ewGwxI-yWAo)

</div>

---

## 🎯 Objectives

By the end of this task, you should be able to:

✅ Operate the robot in gravity compensation mode
✅ Parse joint positions from the `/joint_states` topic
✅ Compute forward kinematics using either **KDL** or **DH parameters**
✅ Publish the real-time end-effector pose as a `PoseStamped` message
✅ Visualize or log the computed end-effector trajectory

---

## 🧠 Background

### 🔧 Gravity Compensation Mode

In this mode, the robot disables joint-level torques, allowing the user to move the robot manually by hand. This is ideal for demonstration, pose logging, and motion capture.

You will move the robot and your script will compute and publish the corresponding **6-DoF end-effector pose** using the joint values received from `/joint_states`.

---

### 🧮 Forward Kinematics Methods

You may implement FK using either of the two methods below:

#### ✅ Option 1: KDL (Recommended)

Leverage the `kdl_parser_py` and `PyKDL` libraries to extract the kinematic chain from the URDF and compute FK automatically.

```python
fk_solver = kdl.ChainFkSolverPos_recursive(kdl_chain)
fk_solver.JntToCart(q, frame)  # Gives pose as KDL::Frame
```

#### ✅ Option 2: Manual DH Table

If you want a deeper understanding of the math, extract the DH parameters of your robot, construct the transformation matrices manually, and compute the final pose.

```python
T = T1 @ T2 @ ... @ Tn  # Product of individual link transforms
```

---

## 📁 Files to Work With

### 1. Template File

```bash
arora_summer_school/challenges/02_fk_gravity_comp_template.py
```

### 2. Launch URDF

Make sure your robot’s URDF is loaded via:

```bash
roslaunch your_robot_description upload.launch
```

---

## 🛠️ How to Run

```bash
rosrun your_package 02_fk_gravity_comp_template.py
```

The node:

* Subscribes to `/joint_states`
* Publishes to `/end_effector_pose`

---

## ✏️ Sections to Complete

### ✅ 1. Joint-State to FK Conversion

You need to compute the end-effector pose in one of two functions:

* `compute_fk_with_kdl(msg)` — implemented for you
* `compute_fk_with_dh(msg)` — **your task to complete**

### ✅ 2. Publishing the Pose

Both methods should return a `geometry_msgs/PoseStamped` object which is then published by the node.

---

## 🔍 Debugging and Visualization

You can visualize the result using:

```bash
rosrun tf tf_echo base_link tool_ff
rostopic echo /end_effector_pose
```

Or plot the 3D trajectory using your custom plotting script.

---

## ✅ Checklist

* [ ] Gravity compensation is active.
* [ ] URDF is published to `/robot_description`.
* [ ] FK node receives `/joint_states` in correct order.
* [ ] End-effector pose is published to `/end_effector_pose`.
* [ ] You understand **both** KDL and DH methods.

---

## 💡 Pro Tips

* KDL is more robust and easy to use but hides the math.
* DH helps you learn about matrix-based FK — useful for oral exams.
* Ensure joint names from `/joint_states` match those in your URDF.
* Set `frame_id` of `PoseStamped` to `"base_link"` or your actual base frame.
* Use `rosparam` to toggle between KDL and DH if you want runtime flexibility.

---

## 🏁 Submission Requirements

* ✅ `02_fk_gravity_comp_template.py` completed and working
* ✅ Use either FK method
* ✅ Publish correct pose format
* ✅ Short GIF or video showing live RViz or console output

---
