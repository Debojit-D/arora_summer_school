# 🚢 Navy Challenge — Laser‑Tip World Pose & Plane‑Intersection

---

## 📄 Overview

A laser module is mounted on the **HEAL** robot’s tool flange.
Your mission is to compute, in real time,

1. **Laser origin / tip** in the **world frame**, and
2. The **(x ,y) intersection** of the laser beam with a horizontal plane (e.g., a table‑top).

You will complete a ROS‑Python script that:

* subscribes to the published **end‑effector pose** (`/end_effector_pose`)
* applies a **fixed tool → laser transform** (RPY + XYZ)
* publishes two `PointStamped` topics:

  * `/laser_tip_world` — laser origin / tip in world coordinates
  * `/laser_plane_intersection` — beam hit‑point on the plane `z = H`

---

## 🎯 Learning Objectives

By completing this task you will:

✅ Manipulate homogeneous transforms (4 × 4 matrices)
✅ Convert **PoseStamped → matrix** and vice‑versa
✅ Understand laser–frame vs. tool–frame relationships
✅ Compute the parametric intersection of a ray with a plane
✅ Publish real‑time geometric data for downstream use (e.g., targeting, mapping)

---

## 🗂️ Files & Folders

| Path                                             | Purpose                                        |
| ------------------------------------------------ | ---------------------------------------------- |
| `challenges/05_laser_world_publisher_student.py` | **Student template** you must complete         |
| `solutions/05_laser_world_publisher_solution.py` | Reference solution *(view only after trying!)* |
| `docs/task_5_navy_laser.md`                      | This detailed README                           |
| other tasks …                                    | —                                              |

---

## 🛠️ Prerequisites

* **Task 02** FK node (publishing `/end_effector_pose`) is running.
* Robot URDF is uploaded (`/robot_description`).
* `/joint_states` and your velocity controller are active.
* Basic understanding of homogeneous transforms and Euler/RPY.

---

## 🧩 Sections to Complete

Open **`05_laser_world_publisher_student.py`** and implement every **`TODO`** block:

| #  | Function / Block                    | What to do                                            |
| -- | ----------------------------------- | ----------------------------------------------------- |
|  1 |  Build `self.T_tool_laser`          | Convert param RPY (deg) → rotation; assign XYZ offset |
|  2 |  `T_w_tool` from PoseStamped        | Pose → 4 × 4 matrix                                   |
|  3 |  Compute `T_w_laser`                | Matrix multiplication                                 |
|  4 |  Laser origin & tip in world coords | Homogeneous point transform                           |
|  5 |  Beam direction (`+X` axis)         | First column of rotation matrix                       |
|  6 |  Plane‑intersection math            | Solve for *t* where `z = plane_height`                |
|  7 |  Publish `PointStamped` messages    | Fill header, xyz; call `publish()`                    |

---

## 🔧 Running the Template

```bash
# Terminal 1 – FK node (Task 02)
rosrun your_pkg 02_fk_gravity_comp_student.py

# Terminal 2 – Laser task
rosrun your_pkg 05_laser_world_publisher_student.py \
        _laser_rpy:="[90,0,0]" \
        _laser_offset_xyz:="[0.0,0.0,0.05]" \
        _tip_distance:=0.0 \
        _plane_height:=0.0
```

* Adjust parameters to match your hardware mount.
* View outputs:

```bash
rostopic echo /laser_tip_world
rostopic echo /laser_plane_intersection
```

* Visualise in RViz: **Add → PointStamped**, select the topics.

---

## 📜 Mathematical Notes

### 1. Tool → Laser Transform

`T_tool_laser` = `R(roll,pitch,yaw) ⨉ Trans(offset_xyz)`

### 2. World → Laser

`T_w_laser = T_w_tool · T_tool_laser`

### 3. Beam Direction

`beam_dir_world = T_w_laser[:3, 0]`   # +X axis

### 4. Ray / Plane Intersection

For origin **o** and direction **d** (unit or not):

```
o_z + t * d_z = plane_height  →  t = (H - o_z) / d_z
```

Use intersection only if `|d_z| > ε` **and** `t ≥ 0`.

---

## 🛠️ Troubleshooting

| Symptom                      | Likely Cause                                | Fix                                                |
| ---------------------------- | ------------------------------------------- | -------------------------------------------------- |
| `/laser_tip_world` all zeros | `TODO` blocks not filled or transform wrong | Check matrices; print debug                        |
| No intersection published    | Beam parallel to plane or pointing away     | Verify RPY; ensure `d_z ≠ 0` and `t ≥ 0`           |
| Wrong XYZ values             | Offset or RPY wrong‑handed                  | Verify laser mount dimensions and sign conventions |

---

## 🏁 Submission Checklist

* [ ] All `TODO` blocks implemented.
* [ ] Node publishes both topics with plausible data.

Good luck, cadet — may your laser hit the bullseye every time! ⚓🔴
