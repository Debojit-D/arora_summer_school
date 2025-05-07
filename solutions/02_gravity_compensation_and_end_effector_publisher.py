#!/usr/bin/env python3
"""
KDL Forward Kinematics Solution Node
Author: Debojit Das (2025)

Overview:
---------
This script demonstrates how to compute and publish the forward kinematics
(end-effector pose) of a robot using the KDL library in ROS.

Scenario:
---------
The user operates the robot in **gravity compensation / free-drive mode**,
moving the robot manually. This node subscribes to `/joint_states`, computes
the real-time end-effector pose using KDL, and publishes it to a topic.

Learning Objective:
-------------------
This script serves as a **reference solution**. Users may either:
1. Use KDL to compute FK as shown here.
2. OR extract the DH parameters from the URDF and compute FK manually.

Topic Summary:
--------------
- Subscribed: `/joint_states` (sensor_msgs/JointState)
- Published:  `/end_effector_pose` (geometry_msgs/PoseStamped)
"""

import time
import rospy
import PyKDL as kdl
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam

class KDLForwardKinematicsNode:
    def __init__(self):
        # --- Load URDF and Build KDL Tree ---
        if not rospy.has_param("robot_description"):
            rospy.logerr("Parameter 'robot_description' not set on parameter server.")
            exit(1)

        self.robot = URDF.from_parameter_server()
        success, self.tree = treeFromParam("robot_description")
        if not success:
            rospy.logerr("❌ Failed to construct KDL tree from URDF")
            exit(1)

        # --- Define Base and Tip Links (change if needed) ---
        base_link = "base_link"
        tip_link = "tool_ff"

        # --- Extract KDL Chain ---
        self.chain = self.tree.getChain(base_link, tip_link)
        self.n_joints = self.chain.getNrOfJoints()
        rospy.loginfo("✅ KDL chain created with %d movable joints.", self.n_joints)

        # --- Create FK Solver (Position Only) ---
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        # --- Extract Joint Names (excluding fixed joints) ---
        self.joint_names = []
        for i in range(self.chain.getNrOfSegments()):
            segment = self.chain.getSegment(i)
            joint = segment.getJoint()
            if joint.getTypeName() != "None":
                self.joint_names.append(joint.getName())

        # --- ROS Communication Setup ---
        self.pose_pub = rospy.Publisher("end_effector_pose", PoseStamped, queue_size=10)
        rospy.Subscriber("joint_states", JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        """Computes FK using the current joint state and publishes the pose."""
        q = kdl.JntArray(self.n_joints)

        # Fill in joint values in correct order
        for i, joint_name in enumerate(self.joint_names):
            try:
                index = msg.name.index(joint_name)
                q[i] = msg.position[index]
            except ValueError:
                rospy.logwarn("Joint '%s' not found in JointState; defaulting to 0", joint_name)
                q[i] = 0.0

        # --- Compute FK ---
        end_effector_frame = kdl.Frame()
        if self.fk_solver.JntToCart(q, end_effector_frame) < 0:
            rospy.logerr("❌ Forward kinematics computation failed")
            return

        # --- Convert KDL Frame to PoseStamped ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"  # Adjust as needed

        # Position
        pose_msg.pose.position.x = end_effector_frame.p[0]
        pose_msg.pose.position.y = end_effector_frame.p[1]
        pose_msg.pose.position.z = end_effector_frame.p[2]

        # Orientation (quaternion)
        qx, qy, qz, qw = end_effector_frame.M.GetQuaternion()
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        # --- Publish Pose ---
        self.pose_pub.publish(pose_msg)
        rospy.logdebug("📡 Published end-effector pose: %s", pose_msg)

if __name__ == '__main__':
    rospy.init_node("kdl_forward_kinematics_node")
    node = KDLForwardKinematicsNode()
    rospy.loginfo("🟢 KDL Forward Kinematics Node Started.")
    rospy.spin()
