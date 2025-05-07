#!/usr/bin/env python3
"""
Navy Challenge – Laser World Pose & Plane Intersection Publisher
Author : Debojit Das (2025)

This node:
1. Subscribes to `/end_effector_pose` (PoseStamped in world frame).
2. Applies a fixed transform (tool → laser) to get the laser frame.
3. Publishes:
   • `/laser_tip_world`          – origin (or tip) of the laser in world frame
   • `/laser_plane_intersection` – beam hit‑point on a horizontal plane (z = H)

ROS Parameters (with defaults)
-------------------------------
~laser_rpy            : [0, 0, 0]   # roll‑pitch‑yaw of laser frame w.r.t tool (deg)
~laser_offset_xyz     : [0, 0, 0]   # XYZ offset of laser origin from tool (m)
~tip_distance         : 0.0         # distance along +X (laser frame) to the tip
~plane_height         : 0.0         # z‑height of target plane for intersection
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import quaternion_matrix, quaternion_from_euler

# --------------------------------------------------------------------------- #
def rpy_deg_to_matrix(rpy_deg):
    """Convert [roll, pitch, yaw] in degrees → 4×4 homogeneous rotation matrix."""
    r, p, y = np.radians(rpy_deg)
    quat = quaternion_from_euler(r, p, y)  # (x, y, z, w)
    return quaternion_matrix(quat)

# --------------------------------------------------------------------------- #
class LaserWorldPublisher:
    def __init__(self):
        # ─── Parameters ──────────────────────────────────────────────────────
        self.rpy_deg        = rospy.get_param("~laser_rpy",            [0.0, 0.0, 0.0])
        self.offset_xyz     = rospy.get_param("~laser_offset_xyz",     [0.0, 0.0, 0.0])
        self.tip_distance   = rospy.get_param("~tip_distance",         0.0)
        self.plane_height   = rospy.get_param("~plane_height",         0.0)

        # Build constant transform T(tool→laser)
        T_tool_laser             = rpy_deg_to_matrix(self.rpy_deg)
        T_tool_laser[0:3, 3]     = self.offset_xyz
        self.T_tool_laser        = T_tool_laser  # 4×4

        # ─── Publishers & Subscriber ────────────────────────────────────────
        self.tip_pub   = rospy.Publisher("/laser_tip_world",          PointStamped, queue_size=10)
        self.hit_pub   = rospy.Publisher("/laser_plane_intersection", PointStamped, queue_size=10)
        rospy.Subscriber("/end_effector_pose", PoseStamped, self.ee_cb)

        rospy.loginfo("Laser publisher initialised:")
        rospy.loginfo("  laser_rpy_deg       : %s", self.rpy_deg)
        rospy.loginfo("  laser_offset_xyz    : %s", self.offset_xyz)
        rospy.loginfo("  tip_distance (m)    : %.3f", self.tip_distance)
        rospy.loginfo("  plane_height (m)    : %.3f", self.plane_height)

    # --------------------------------------------------------------------- #
    def ee_cb(self, pose_msg):
        """Compute laser origin & plane‑intersection from EE pose."""
        # PoseStamped → homogeneous transform T(world→tool)
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        T_w_tool         = quaternion_matrix([q.x, q.y, q.z, q.w])
        T_w_tool[0:3, 3] = [p.x, p.y, p.z]

        # Tool → Laser
        T_w_laser = T_w_tool @ self.T_tool_laser

        # Laser origin (homogeneous)
        origin_local = np.array([0.0, 0.0, 0.0, 1.0])
        origin_world = T_w_laser @ origin_local

        # Tip (optional offset along +X)
        tip_local    = np.array([self.tip_distance, 0.0, 0.0, 1.0])
        tip_world    = T_w_laser @ tip_local

        # Beam direction in world frame is +X axis of laser frame
        beam_dir_world = T_w_laser[0:3, 0]  # first column of rotation matrix

        # ── Publish laser tip/origin point ─────────────────────────────────
        pt_msg = PointStamped()
        pt_msg.header = pose_msg.header          # keep same stamp / frame
        pt_msg.point.x, pt_msg.point.y, pt_msg.point.z = tip_world[0:3]
        self.tip_pub.publish(pt_msg)

        # ── Compute intersection with horizontal plane z = plane_height ────
        dir_z = beam_dir_world[2]
        if abs(dir_z) < 1e-6:
            rospy.logdebug("Beam parallel to plane; no intersection.")
            return

        t = (self.plane_height - origin_world[2]) / dir_z
        if t < 0:
            rospy.logdebug("Beam points away from plane; no intersection.")
            return

        hit_world = origin_world[0:3] + t * beam_dir_world  # 3‑vector

        hit_msg = PointStamped()
        hit_msg.header = pose_msg.header
        hit_msg.point.x, hit_msg.point.y, hit_msg.point.z = hit_world
        self.hit_pub.publish(hit_msg)

        rospy.logdebug("Laser tip   : (%.3f, %.3f, %.3f)",
                       tip_world[0], tip_world[1], tip_world[2])
        rospy.logdebug("Plane hit @z=%.3f : (%.3f, %.3f)",
                       self.plane_height, hit_world[0], hit_world[1])

# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    rospy.init_node("laser_world_publisher")
    LaserWorldPublisher()
    rospy.loginfo("🟢 Laser world‑frame publisher running.")
    rospy.spin()
