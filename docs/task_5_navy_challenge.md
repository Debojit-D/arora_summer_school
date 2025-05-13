# 🚢 Navy Challenge — Laser‑Tip World Pose & Plane‑Intersection

---

## 📄 Overview
A laser module is rigidly mounted on the **HEAL** robot’s tool flange.  
Your mission is to compute, in real time,

1. the **laser tip** (origin of the beam) in the **world frame**, and  
2. the **(x, y) coordinate** where the beam hits a horizontal plane  
   *(e.g. the table‑top at height `z = H`)*.

A ROS‑Python node must:

- subscribe to the published **end‑effector pose** (`/end_effector_pose`),
- apply a **fixed tool → laser transform** (RPY + XYZ),
- and publish two `geometry_msgs/PointStamped` topics:

| Topic | Meaning |
|-------|---------|
| `/laser_tip_world` | Laser tip in world coordinates |
| `/laser_plane_intersection` | Beam hit‑point on the plane `z = H` |

---

## 🎯 Learning Objectives
By completing this task you will:

✅ manipulate homogeneous transforms (4 × 4 matrices)  
✅ convert **PoseStamped → matrix** and back  
✅ understand laser‑frame vs tool‑frame relationships  
✅ compute the parametric intersection of a ray with a plane  
✅ publish real‑time geometric data for downstream use (targeting, mapping)

---

## 🗂️ Files & Folders

| Path                                           | Purpose |
|-----------------------------------------------|---------|
| `challenges/05_laser_world_publisher_student.py` | **Student template** you must complete |
| `solutions/05_laser_world_publisher_solution.py` | Reference implementation *(view only after trying!)* |
| `docs/task_5_navy_laser.md`                    | **This README** |
| … other tasks …                                | — |

---

## 🛠️ Prerequisites
* **Task 02** FK node is running (publishes `/end_effector_pose`).
* Robot URDF uploaded on the parameter server (`/robot_description`).
* `/joint_states` and a velocity controller are active.
* Familiarity with homogeneous transforms and Euler/RPY angles.

---

## 🧩 Sections to Complete
Open **`05_laser_world_publisher_student.py`** and implement every **`TODO`**:

| # | Function / Block | What to do |
|---|------------------|----------------------------------------------|
| 1 | Build `self.T_tool_laser` | Convert param RPY (deg) → rotation; assign XYZ offset |
| 2 | `T_w_tool` from PoseStamped | Pose → 4 × 4 matrix |
| 3 | Compute `T_w_laser` | Matrix multiplication |
| 4 | Laser tip in world coords | Homogeneous point transform |
| 5 | Beam direction (`+X` axis) | First column of rotation matrix |
| 6 | Plane‑intersection math | Solve for *t* where `z = plane_height` |
| 7 | Publish `PointStamped` messages | Fill header & xyz; call `publish()` |

---

## 🔧 Running the Template

```bash
# Terminal 1 – FK node (Task 02)
python3 02_gravity_compensation_student_template.py

# Terminal 2 – Laser task
python3 05_laser_world_publisher_student.py \
        _laser_rpy:="[90,0,0]" \
        _laser_offset_xyz:="[0.0,0.0,0.05]" \
        _tip_distance:=0.020 \
        _plane_height:=0.750
```

*Adjust parameters to match **your** hardware mount.*

### Visualise

```bash
rostopic echo /laser_tip_world
rostopic echo /laser_plane_intersection
```

In RViz: *Add → PointStamped* and select the topics.

---

## 📜 Mathematical Notes
Note : If this is not formatted correctly in GitHub ReadMe open this file in VS Code to view it properly.

### 1. Tool → Laser Transform

A constant 4 × 4 matrix:

```math
T^{\text{tool}}_{\text{laser}}
= R(\text{roll},\text{pitch},\text{yaw})\; \cdot\;
  \operatorname{Trans}(\text{offset}_{xyz})
```

### 2. World → Laser

```math
T^{w}_{\text{laser}} = T^{w}_{\text{tool}} \cdot T^{\text{tool}}_{\text{laser}}
```

### 3. Beam Direction

The +X axis of the laser frame:

```math
\mathbf{d} = T^{w}_{\text{laser}}[0:3,\; 0]
```

### 4. Ray / Plane Intersection

Let **o** be the **laser tip** (after applying any positive `tip_distance`)
and **d** be the beam direction (unit or not).
For the horizontal plane $z = H$:

```math
o_z + t\,d_z = H
\quad\Longrightarrow\quad
t = \frac{H - o_z}{d_z}
```

Use the hit‑point only if
$\lvert d_z \rvert > \varepsilon$ **and** $t \ge 0$.

---

## 🛠️ Troubleshooting

| Symptom                           | Likely Cause                                                                 | Fix                                                           |
| --------------------------------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------- |
| `/laser_tip_world` ≡ 0            | A `TODO` not filled or transform badly initialised                           | Print matrices; verify RPY & offset signs                     |
| No hit‑point published            | Beam parallel to plane or pointing away                                      | Check `laser_rpy`; ensure `d_z ≠ 0` **and** `t ≥ 0`           |
| Hit‑point shifted by tip distance | Accidentally used **laser base** instead of **laser tip** for the ray origin | Use `tip_distance` correctly (the solution code already does) |
| Wrong X / Y values                | Offset or RPY sign reversed                                                  | Double‑check mount drawing & RPY convention                   |

---

## 🏁 Submission Checklist

* [ ] All `TODO` blocks implemented.
* [ ] Node publishes **plausible** `/laser_tip_world` and `/laser_plane_intersection`.
* [ ] You tested with various tool poses to confirm correctness.

---

Good luck, cadet — may your laser hit the bullseye every time! ⚓🔴
