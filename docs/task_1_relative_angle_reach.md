
# 🦾 Relative Joint Angle Reaching Task — Student Documentation

## 📄 Overview

This introductory task helps you build **intuition about robot joint angles**. You will be given a **random joint configuration** relative to the current posture of the robot. Your job is to manually drive the robot to the **target joint angles** using a **teleoperation keyboard interface**.

You will then receive a **score** based on how accurately you matched each joint. This is a foundational exercise before moving on to trajectory tracking or Cartesian control.

<div align="center">

[![Watch the video](https://img.youtube.com/vi/QQjjRGbvd3g/hqdefault.jpg)](https://youtu.be/QQjjRGbvd3g)

</div>

---

## 🎯 Objectives

By the end of this task, you will:

✅ Understand the effect of each joint on robot posture
✅ Use joint-level teleoperation for pose control
✅ Match robot joint angles to target configurations
✅ Receive real-time accuracy feedback and scoring

---

## 🧠 Background Concepts

### 1. **Joint-Space Control**

Robots like HEAL are controlled using **joint positions** or **velocities**. Each joint contributes to the robot’s posture in a nonlinear way. Understanding this relationship is critical for tasks like planning, tracking, and manipulation.

### 2. **Relative Targeting**

In this task, the target joint configuration is **generated randomly** with respect to the **current joint angles**. The challenge is to mentally and visually align the robot to reach that configuration using keyboard inputs.

### 3. **Joint Teleoperation**

The robot is moved using a keyboard interface (`joint_teleop_heal.py`), which sends velocity commands per joint. Key bindings follow a simple pattern:

| Key       | Joint   | Direction |
| --------- | ------- | --------- |
| `q` / `a` | Joint 1 | + / –     |
| `w` / `s` | Joint 2 | + / –     |
| `e` / `d` | Joint 3 | + / –     |
| `r` / `f` | Joint 4 | + / –     |
| `t` / `g` | Joint 5 | + / –     |
| `y` / `h` | Joint 6 | + / –     |

---

## 📁 File Location

```bash
arora_summer_school/challenges/01_relative_angle_reaching.py
```

You will also use the joint teleoperation interface:

```bash
arora_summer_school/utils/joint_teleop_heal.py
```

---

## 🧰 Steps to Perform

### 🟢 1. Launch Robot

Ensure that:

* The robot URDF is loaded on the parameter server (`/robot_description`)
* The `/joint_states` topic is publishing
* The velocity controller is active on `/velocity_controller/command`

### 🧑‍💻 2. Run the Task Script

In the terminal go to the path where the file exits and type:

```bash
python3 01_relative_angle_reaching.py
```

You will see a **random joint configuration** displayed.

```bash
🎯 Target Joint Angles:
   Joint 1: -32°
   Joint 2: +12°
   Joint 3: +45°
   Joint 4: -67°
   Joint 5: +18°
   Joint 6: +02°
```

### 🎮 3. Run the Teleop Controller

In a **second terminal**, navigate to the utils folder and run:

```bash
python3 joint_teleop_heal.py
```

Use the keys to move each joint **individually** toward the target. Press `ESC` to stop.

---

### ✅ 4. Confirm Your Attempt

Return to the task script terminal and press `[Enter]` when you think the robot has matched the pose.

You will then see a **performance breakdown**:

```
📊 Performance Summary:
   - Joint 1: Error = 3.25° (Overshoot ➡️), Score = 7/10
   - Joint 2: Error = 1.12° (Perfect 🎯), Score = 10/10
   ...
🏅 Total Score: 53/60
🏆 SUCCESS!
```

The system uses joint-wise **percentage-based scoring** depending on the joint range.

| % Error of Joint Range | Score |
| ---------------------- | ----- |
| > 50%                  | 0     |
| 30–50%                 | 3     |
| 20–30%                 | 5     |
| 10–20%                 | 7     |
| ≤ 10%                  | 10    |

---

### 🤖 5. Auto-Move and Home Return

After evaluation, the script asks:

```bash
🤖 Auto-move robot to target? (y/n):
```

* If you say `yes`, the robot will move smoothly to the target using a velocity trajectory.
* After that, it will return to **home position** (all joint angles = 0°).

---

## 📦 Internals (What's Happening Behind the Scenes)

* `get_current_joint_angles_deg()`: Listens to `/joint_states` and returns joint positions.
* `generate_target_angles()`: Creates target angles using per-joint limits.
* `evaluate_performance()`: Computes per-joint signed errors and scores.
* `move_robot_to_joint_angles()`: Plans a **quintic velocity profile** to the target using `TrajectoryPlanner`.

---

## 🛠️ Troubleshooting

| Problem                      | Cause                                | Fix                                   |
| ---------------------------- | ------------------------------------ | ------------------------------------- |
| Joint positions not updating | Teleop not sending velocities        | Check teleop terminal and key presses |
| Target not reached           | Manual driving too far or inaccurate | Retry using smaller velocity steps    |
| FK looks off                 | Wrong base or tool link in URDF      | Verify URDF and robot config          |

---

Happy reaching, and enjoy mastering your joints! 🦾
