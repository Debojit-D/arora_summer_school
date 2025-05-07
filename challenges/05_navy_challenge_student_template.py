#!/usr/bin/env python3
"""
STUDENT TEMPLATE  – Laser World‑Frame Publisher
================================================

Goal
----
  ▸ Subscribe to `/end_effector_pose` (PoseStamped, world frame).  
  ▸ Apply the fixed tool‑→laser transform (RPY + XYZ).  
  ▸ Publish:
      • `/laser_tip_world`          – laser origin / tip in world frame  
      • `/laser_plane_intersection` – point where the beam meets a plane z = H  

Complete every `TODO` block.  
You may verify with  `rostopic echo /laser_plane_intersection`
and RViz.

Parameters (rosparam)
---------------------
~laser_rpy            : [roll, pitch, yaw] (deg)     – laser orientation w.r.t. tool  
~laser_offset_xyz     : [x, y, z] (m)                – translation tool → laser origin  
~tip_distance         : float (m)                    – distance along +X (laser) to tip  
~plane_height         : float (m)                    – Z height of target plane
"""

# ─────────────────────────────────────────────────────────────────────────────
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import (
    quaternion_from_euler, quaternion_matrix
)

# ─────────────────────────────────────────────────────────────────────────────
def rpy_deg_to_matrix(rpy_deg):
    """Convert RPY degrees → 4×4 homogeneous matrix."""
    r, p, y = np.radians(rpy_deg)
    quat = quaternion_from_euler(r, p, y)          # x, y, z, w
    return quaternion_matrix(quat)                 # 4×4

# ─────────────────────────────────────────────────────────────────────────────
class LaserPublisherStudent:
    def __init__(self):
        # ---------- 1. Read parameters ----------
        self.rpy_deg      = rospy.get_param("~laser_rpy",            [0., 0., 0.])
        self.offset_xyz   = rospy.get_param("~laser_offset_xyz",     [0., 0., 0.])
        self.tip_distance = rospy.get_param("~tip_distance",         0.0)
        self.plane_height = rospy.get_param("~plane_height",         0.0)

        # ---------- 2. Build constant T(tool→laser) ----------
        # TODO: build self.T_tool_laser (4×4) using rpy_deg_to_matrix()
        #       and self.offset_xyz
        # HINT:  mat[:3,3] = offset
        self.T_tool_laser = None  # <- replace

        # ---------- 3. ROS I/O ----------
        self.tip_pub = rospy.Publisher("/laser_tip_world",
                                       PointStamped, queue_size=10)
        self.hit_pub = rospy.Publisher("/laser_plane_intersection",
                                       PointStamped, queue_size=10)

        rospy.Subscriber("/end_effector_pose",
                         PoseStamped, self.ee_cb)

        rospy.loginfo("Laser node ready – fill in TODOs to compute FK.")

    # --------------------------------------------------------------------- #
    def ee_cb(self, pose_msg):
        """
        Called on every end‑effector PoseStamped.
        1. Build T(world→tool)
        2. Compute laser origin & direction in world frame
        3. Publish /laser_tip_world
        4. Intersect beam with horizontal plane z = plane_height
        """
        # ---------- 3a. World→Tool transform ----------
        # TODO: convert pose_msg → homogeneous matrix T_w_tool (4×4)
        #       Position in translation; orientation via quaternion_matrix().
        T_w_tool = None  # <- replace with correct matrix

        # ---------- 3b. Tool→Laser ----------
        # TODO: combine transforms to get T_w_laser
        T_w_laser = None  # <- replace

        # ---------- 3c. Compute laser tip (origin + tip_distance along +X) ----------
        # origin_world = T_w_laser @ [0,0,0,1]
        # tip_world    = T_w_laser @ [tip_distance,0,0,1]
        origin_world = None  # <- replace (4×1)
        tip_world    = None  # <- replace (4×1)

        # TODO: beam_dir_world = +X axis of laser frame (first column of R)
        beam_dir_world = None  # <- replace (3,)

        # ---------- 3d. Publish laser tip ----------
        tip_msg = PointStamped()
        tip_msg.header = pose_msg.header
        tip_msg.point.x, tip_msg.point.y, tip_msg.point.z = tip_world[:3] \
            if tip_world is not None else (0, 0, 0)
        self.tip_pub.publish(tip_msg)

        # ---------- 3e. Intersection with plane z = plane_height ----------
        #  Solve  origin.z + t * dir.z = plane_height  → t
        #  Check |dir.z| > eps and t ≥ 0 (beam forward)
        intersection_ok = False  # set True after computing

        # TODO: compute intersection and publish if valid
        # hit_msg = PointStamped();  fill header & xyz ; self.hit_pub.publish(hit_msg)

        if not intersection_ok:
            rospy.logdebug("No valid plane intersection computed.")

# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    rospy.init_node("laser_world_publisher_student")
    LaserPublisherStudent()
    rospy.loginfo("🟢 Laser publisher student node started. Complete the TODOs!")
    rospy.spin()
