# 📈 Trajectory Tracking Task — Student Documentation

## 📄 Overview

This task focuses on implementing **Cartesian trajectory tracking** for the **HEAL robotic arm** using a custom **Damped Least Squares (DLS)** inverse kinematics controller. You will complete a Python script that allows the robot to follow various motion patterns:

* Circular motion while maintaining tangential orientation (like holding a ring).
* Sine wave traversal with return path along the same curve.
* Straight-line sweeps in task space.

You will edit `trajectory_tracking_student_template.py` to implement these trajectory functions and visualize the path.

<div align="center">

[![Watch the video](https://img.youtube.com/vi/x2lWNBFbgVk/hqdefault.jpg)](https://youtu.be/x2lWNBFbgVk)

</div>

---

## 🎯 Objectives

By the end of this task, you should be able to:

✅ Write trajectory generator functions in Cartesian space
✅ Track dynamic targets using a DLS-based velocity controller
✅ Maintain proper orientation of the end-effector
✅ Log and visualize 3D Cartesian paths

---

## 🧠 Background Concepts

### 1. **Damped Least Squares (DLS) IK**

DLS avoids singularities by computing joint velocities:

$\dot{q} = J^T (JJ^T + \lambda^2 I)^{-1} \xi$

Where:

* $J$: Jacobian matrix
* $\xi$: Cartesian velocity (linear + angular)
* $\lambda$: damping factor

> **Note:** Angular velocity control is **disabled** by default for safety (cable winding risk). You can enable it in the controller file.

### 2. **Trajectory Planning**

Each trajectory returns a 6D target pose:

* `pos`: Cartesian position \[x, y, z]
* `quat`: Orientation as quaternion \[x, y, z, w]

These are updated every 10 ms and fed to the velocity controller.

### 3. **Orientation Modes**

* **Fixed Downward**: EE z-axis points down (e.g., for waving, sweeping).
* **Tangential Circular**: For ring-holding, orientation follows trajectory direction.

---

## 📁 File Location

```bash
arora_summer_school/challenges/trajectory_tracking_student_template.py
```

Run the script using:

```bash
rosrun your_package trajectory_tracking_student_template.py
```

---

## 🧰 Sections to Complete

### ✏️ 1. Circular Trajectory

In the function `circular_trajectory(t, ...)`, complete the following:

```python
# TODO: Compute x, y, z based on omega*t
# TODO: Compute tangent vector and construct dynamic orientation
# TODO: Return pos, quat
```

| Requirement       | Tip                                      |
| ----------------- | ---------------------------------------- |
| Tangential motion | Use velocity vector along the circle     |
| Orientation       | Construct rotation matrix (x, y, z axes) |

---

### ✏️ 2. Sine Wave Bidirectional

Edit `sine_wave_bidirectional(t)`:

```python
# TODO: Compute x from sine, and y using sin(pi * x)
# TODO: Reverse time direction every period to trace backward
```

| Requirement       | Tip                                    |
| ----------------- | -------------------------------------- |
| Smooth reversal   | Use modulo + sign flip with `np.sin()` |
| Return path match | Keep same y profile during reverse     |

---

### ✏️ 3. Straight-Line Sweep

In `straight_line_trajectory(t)`:

```python
# TODO: Use sine to oscillate between x_min and x_max
```

| Requirement      | Tip                                |
| ---------------- | ---------------------------------- |
| Constant Y and Z | Fix values to default robot height |
| Smoothness       | Use `np.sin()` to vary x           |

---

### ✏️ 4. Custom Trajectory

Implement `custom_trajectory(t)` with your own logic:

```python
# TODO: Design your own pos, quat values over time
```

Examples:

* Helical motion
* Zigzag
* Arc segment

---

## 🔮 How to Test

### ✅ Requirements:

* URDF loaded into `/robot_description`
* `/joint_states` being published
* Active velocity controller on `/velocity_controller/command`

### 🚀 Run the script:

```bash
rosrun your_package trajectory_tracking_student_template.py
```

You should see:

* Smooth Cartesian motion following selected trajectory
* Logged path visualization after shutdown

---

## 🛠️ Troubleshooting

| Problem                 | Possible Cause                     | Fix                                       |
| ----------------------- | ---------------------------------- | ----------------------------------------- |
| No motion               | Joint states not received          | Verify `/joint_states` topic              |
| Orientation flickers    | Quaternion sign flips              | Check hemisphere continuity logic         |
| Robot shakes or jitters | Too high velocity or damping issue | Limit velocity or increase damping factor |
| Path looks distorted    | Incorrect math in trajectory func  | Review trajectory logic and time base     |

---

## 🔧 Pro Tips

* Use `rostopic echo /joint_states` to debug inputs.
* Enable angular motion **only when required**.
* Visualize the robot in RViz to observe orientation.
* Save plots to files using `plt.savefig("traj.png")` if needed.

---

Good luck, and enjoy making the robot move beautifully! 🧠
