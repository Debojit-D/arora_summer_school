#!/usr/bin/env python3
"""
Navy Challenge – Laser World Pose & Plane Intersection Publisher
Author : Debojit Das (2025)  |  Patch by ChatGPT‑o3 (2025‑05‑13)

This node:
1. Subscribes to `/end_effector_pose` (PoseStamped, world frame).
2. Applies a fixed transform (tool → laser) to locate the laser frame.
3. Publishes
   • `/laser_tip_world`          – tip of the laser in world frame
   • `/laser_plane_intersection` – beam hit‑point on plane z = H
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import quaternion_matrix, quaternion_from_euler

# --------------------------------------------------------------------------- #
def rpy_deg_to_matrix(rpy_deg):
    """[roll, pitch, yaw] in degrees  →  4 × 4 homogeneous rotation matrix."""
    r, p, y = np.radians(rpy_deg)
    quat = quaternion_from_euler(r, p, y)           # (x, y, z, w)
    return quaternion_matrix(quat)                  # 4 × 4

# --------------------------------------------------------------------------- #
class LaserWorldPublisher:
    def __init__(self):
        # ── Params ---------------------------------------------------------
        self.rpy_deg      = rospy.get_param("~laser_rpy",        [0.0, 0.0, 0.0])
        self.offset_xyz   = rospy.get_param("~laser_offset_xyz", [0.0, 0.0, 0.0])
        self.tip_distance = rospy.get_param("~tip_distance",     0.0)
        self.plane_height = rospy.get_param("~plane_height",     0.0)

        # Tool → laser transform (constant)
        self.T_tool_laser             = rpy_deg_to_matrix(self.rpy_deg)
        self.T_tool_laser[0:3, 3]     = self.offset_xyz          # add translation

        # ── ROS I/O --------------------------------------------------------
        self.tip_pub = rospy.Publisher("/laser_tip_world",
                                       PointStamped, queue_size=10)
        self.hit_pub = rospy.Publisher("/laser_plane_intersection",
                                       PointStamped, queue_size=10)
        rospy.Subscriber("/end_effector_pose", PoseStamped, self.ee_cb)

        # Pretty banner
        rospy.loginfo("🔧 Laser publisher initialised with:")
        rospy.loginfo("    laser_rpy_deg  : %s", self.rpy_deg)
        rospy.loginfo("    laser_offset   : %s  (m)", self.offset_xyz)
        rospy.loginfo("    tip_distance   : %.3f m", self.tip_distance)
        rospy.loginfo("    plane_height   : %.3f m", self.plane_height)

    # --------------------------------------------------------------------- #
    def ee_cb(self, pose_msg: PoseStamped):
        """Handle each incoming end‑effector pose."""
        # PoseStamped → homogeneous transform T(world→tool)
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        T_w_tool         = quaternion_matrix([q.x, q.y, q.z, q.w])
        T_w_tool[0:3, 3] = [p.x, p.y, p.z]

        # Compose with constant tool → laser transform
        T_w_laser = T_w_tool @ self.T_tool_laser

        # 1️⃣ Laser tip in world frame
        tip_local  = np.array([self.tip_distance, 0.0, 0.0, 1.0])
        tip_world  = T_w_laser @ tip_local          # 4‑vec (x, y, z, 1)

        # Publish tip
        tip_msg = PointStamped()
        tip_msg.header = pose_msg.header            # keeps /world frame & time
        tip_msg.point.x, tip_msg.point.y, tip_msg.point.z = tip_world[0:3]
        self.tip_pub.publish(tip_msg)

        # Beam direction = +X axis of laser frame (unit length)
        beam_dir_world = T_w_laser[0:3, 0]          # 3‑vec

        # Guard: beam parallel to plane?
        dir_z = beam_dir_world[2]
        if abs(dir_z) < 1e-6:
            rospy.logdebug("Beam parallel to plane; skipping intersection.")
            return

        # 2️⃣ Intersection with horizontal plane z = plane_height
        t = (self.plane_height - tip_world[2]) / dir_z
        if t < 0:
            rospy.logdebug("Beam points away from plane; skipping.")
            return

        hit_world = tip_world[0:3] + t * beam_dir_world

        hit_msg = PointStamped()
        hit_msg.header = pose_msg.header
        hit_msg.point.x, hit_msg.point.y, hit_msg.point.z = hit_world
        self.hit_pub.publish(hit_msg)

        rospy.logdebug("Tip (world):  %.3f  %.3f  %.3f",
                       tip_world[0], tip_world[1], tip_world[2])
        rospy.logdebug("Hit @ z=%.3f: %.3f  %.3f",
                       self.plane_height, hit_world[0], hit_world[1])

# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    rospy.init_node("laser_world_publisher")
    LaserWorldPublisher()
    rospy.loginfo("🟢 Laser world‑frame publisher running.")
    rospy.spin()
