#!/usr/bin/env python3
"""
KDL Forward Kinematics — Student Template
Author: Debojit Das (2025)

Overview:
---------
This script sets up a node to compute the end-effector pose of a robotic arm using one of two approaches:
1. KDL-based forward kinematics using the robot URDF.
2. DH parameter-based manual forward kinematics (to be implemented by the student).

Instructions:
-------------
💡 Choose ONE of the two FK methods:
- Implement `compute_fk_with_kdl()` using KDL (already provided).
- Implement `compute_fk_with_dh()` using your own DH matrix chain.

The result must be published to `/end_effector_pose` as a `PoseStamped`.

Subscribed Topics:
- `/joint_states` (sensor_msgs/JointState)

Published Topics:
- `/end_effector_pose` (geometry_msgs/PoseStamped)
"""

import rospy
import PyKDL as kdl
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam


class FKStudentNode:
    def __init__(self, use_kdl=True):
        self.use_kdl = use_kdl
        self.n_joints = 6  # Change if your robot has a different number of joints

        # --- Initialize ROS Communication ---
        self.pose_pub = rospy.Publisher("/end_effector_pose", PoseStamped, queue_size=10)
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        # --- Setup for KDL (Optional if using DH) ---
        if use_kdl:
            self.setup_kdl()

    def setup_kdl(self):
        """Loads URDF and builds the KDL chain."""
        if not rospy.has_param("robot_description"):
            rospy.logerr("❌ Parameter 'robot_description' not found.")
            exit(1)

        self.robot = URDF.from_parameter_server()
        success, self.tree = treeFromParam("robot_description")
        if not success:
            rospy.logerr("❌ Could not build KDL tree.")
            exit(1)

        base_link = "base_link"
        tip_link = "tool_ff"
        self.chain = self.tree.getChain(base_link, tip_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        self.joint_names = []
        for i in range(self.chain.getNrOfSegments()):
            joint = self.chain.getSegment(i).getJoint()
            if joint.getTypeName() != "None":
                self.joint_names.append(joint.getName())

        self.n_joints = self.chain.getNrOfJoints()
        rospy.loginfo("✅ KDL chain created with %d joints.", self.n_joints)

    def joint_state_callback(self, msg):
        """Callback that processes incoming joint states and computes FK."""
        if self.use_kdl:
            pose = self.compute_fk_with_kdl(msg)
        else:
            pose = self.compute_fk_with_dh(msg)

        if pose:
            self.pose_pub.publish(pose)

    def compute_fk_with_kdl(self, msg):
        """Computes FK using KDL and returns a PoseStamped."""
        q = kdl.JntArray(self.n_joints)
        for i, joint_name in enumerate(self.joint_names):
            try:
                index = msg.name.index(joint_name)
                q[i] = msg.position[index]
            except ValueError:
                rospy.logwarn("⚠️ Joint '%s' not in JointState; defaulting to 0", joint_name)
                q[i] = 0.0

        frame = kdl.Frame()
        if self.fk_solver.JntToCart(q, frame) < 0:
            rospy.logerr("❌ FK computation failed.")
            return None

        return self.kdl_frame_to_pose(frame)

    def compute_fk_with_dh(self, msg):
        """
        STUDENT TODO: Compute FK manually using your robot's DH parameters.

        - Create transformation matrices for each joint.
        - Chain them together to get the final pose.
        - Convert to a PoseStamped and return.
        """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"

        # Example: You must replace this with real DH-based matrix chain
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        return pose

    def kdl_frame_to_pose(self, frame):
        """Converts KDL::Frame to PoseStamped."""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = frame.p[0]
        pose.pose.position.y = frame.p[1]
        pose.pose.position.z = frame.p[2]
        qx, qy, qz, qw = frame.M.GetQuaternion()
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose


if __name__ == '__main__':
    rospy.init_node("fk_student_node")
    use_kdl_fk = rospy.get_param("~use_kdl", True)  # Set False to use DH manually
    node = FKStudentNode(use_kdl=use_kdl_fk)
    rospy.loginfo("🟢 Forward Kinematics Node Started. Mode: %s", "KDL" if use_kdl_fk else "DH Manual")
    rospy.spin()
