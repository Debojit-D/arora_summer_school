#!/usr/bin/env python3
# File: dls_velocity_commander.py
# Author: Debojit Das (2025)
#
# Description:
# - Damped Least Squares Cartesian velocity control.
# - Uses live joint state feedback and internal forward kinematics.
# - Limits linear and angular velocities.
# - Ensures quaternion hemisphere continuity.
# - Avoids singularities using damping factor.
# -------------------------------------------------------------------------

import os
import sys
import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam

# ------------------------ Utility -----------------------------------------
def normalize_quaternion(q):
    q = np.array(q)
    return q / np.linalg.norm(q)

# ------------------------ Main Class --------------------------------------
class DLSVelocityCommander:
    def __init__(self, target_pos, target_quat, custom_ds=None):
        """
        Parameters:
        - target_pos: [x, y, z] in meters (target end-effector position)
        - target_quat: [x, y, z, w] quaternion (target end-effector orientation)
        - custom_ds: Optional callable to override position control (Dynamical System)
        """
        rospy.init_node("dls_velocity_commander", anonymous=True)

        # ROS publishers and subscribers
        self.pub = rospy.Publisher("/velocity_controller/command",
                                   Float64MultiArray, queue_size=10)
        self.joint_state_sub = rospy.Subscriber("/joint_states",
                                                JointState, self.joint_state_callback)

        # Controller parameters
        self.damping = 0.1                     # Damping factor for DLS pseudoinverse
        self.dt = 0.01                         # Control rate (s)
        self.max_cartesian_vel = 0.05          # Max linear velocity (m/s)
        self.max_angular_vel = 0.1            # Max angular velocity (rad/s)
        self.custom_ds = custom_ds             # Optional dynamical system override

        # Load URDF and create KDL chain
        if not rospy.has_param("/robot_description"):
            rospy.logerr("URDF parameter '/robot_description' not found.")
            exit(1)

        self.robot = URDF.from_parameter_server("/robot_description")
        success, tree = treeFromParam("/robot_description")
        if not success:
            rospy.logerr("Failed to parse KDL tree from robot description.")
            exit(1)

        self.base_link = "base_link"
        self.tip_link = "tool_ff"
        self.chain = tree.getChain(self.base_link, self.tip_link)
        self.n_joints = self.chain.getNrOfJoints()
        rospy.loginfo("✅ Heal chain with %d joints loaded", self.n_joints)

        # Setup KDL solvers
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

        # Target frame
        qx, qy, qz, qw = normalize_quaternion(target_quat)
        self.target_pos = kdl.Vector(*target_pos)
        self.target_quat = kdl.Rotation.Quaternion(qx, qy, qz, qw)
        self.target_frame = kdl.Frame(self.target_quat, self.target_pos)

        self.current_joint_state = None  # Updated by callback

    # ---------------- Joint State Callback ------------------
    def joint_state_callback(self, msg):
        self.current_joint_state = np.array(msg.position)

    # ---------------- Main DLS IK Computation ----------------
    def compute_dls_ik(self, q_current):
        # Compute forward kinematics
        current_frame = kdl.Frame()
        self.fk_solver.JntToCart(q_current, current_frame)

        # Extract position
        current_pos = np.array([current_frame.p.x(),
                                current_frame.p.y(),
                                current_frame.p.z()])

        # Compute linear error (or use DS override)
        if self.custom_ds:
            linear_error = self.custom_ds(current_pos)
        else:
            delta = self.target_frame.p - current_frame.p
            linear_error = np.array([delta.x(), delta.y(), delta.z()])

        # Limit linear velocity
        lin_norm = np.linalg.norm(linear_error)
        if lin_norm > self.max_cartesian_vel:
            linear_error = (linear_error / lin_norm) * self.max_cartesian_vel

        # Quaternion hemisphere continuity
        target_q = np.array(self.target_frame.M.GetQuaternion())
        current_q = np.array(current_frame.M.GetQuaternion())
        if np.dot(target_q, current_q) < 0.0:
            # Flip target quaternion
            target_q = -target_q
            self.target_frame.M = kdl.Rotation.Quaternion(*target_q)

        # Compute angular error (body frame)
        R_err = self.target_frame.M * current_frame.M.Inverse()
        angle, axis = R_err.GetRotAngle()
        angular_error = np.array([axis.x(), axis.y(), axis.z()]) * angle

        # Limit angular velocity
        ang_norm = np.linalg.norm(angular_error)
        if ang_norm > self.max_angular_vel:
            angular_error = (angular_error / ang_norm) * self.max_angular_vel

        # Stack into twist vector [vx, vy, vz, wx, wy, wz]
        twist_error = np.concatenate((linear_error, angular_error*0))

        # Compute Jacobian
        jacobian = kdl.Jacobian(self.n_joints)
        self.jac_solver.JntToJac(q_current, jacobian)
        J = np.array([[jacobian[i, j] for j in range(self.n_joints)] for i in range(6)])

        # Damped Least Squares inverse
        lambda_sq = self.damping ** 2
        try:
            J_dls = J.T @ np.linalg.inv(J @ J.T + lambda_sq * np.eye(6))
        except np.linalg.LinAlgError:
            rospy.logerr("❌ Jacobian inversion failed (singular)! Sending zero velocity.")
            return np.zeros(self.n_joints)

        dq = J_dls @ twist_error
        return dq

    # ---------------- Main Run Loop ------------------------
    def run(self):
        rate = rospy.Rate(1.0 / self.dt)
        while not rospy.is_shutdown():
            if self.current_joint_state is None:
                rate.sleep()
                continue

            # Convert to KDL format
            q_current = kdl.JntArray(self.n_joints)
            for i in range(self.n_joints):
                q_current[i] = self.current_joint_state[i]

            dq = self.compute_dls_ik(q_current)
            msg = Float64MultiArray()
            msg.data = dq.tolist()
            self.pub.publish(msg)
            rate.sleep()

    def run_once(self):
        """
        Perform a single DLS IK update and publish joint velocity.
        Useful for external control loops (e.g., trajectory following).
        """
        if self.current_joint_state is None:
            return

        q_current = kdl.JntArray(self.n_joints)
        for i in range(self.n_joints):
            q_current[i] = self.current_joint_state[i]

        dq = self.compute_dls_ik(q_current)
        msg = Float64MultiArray()
        msg.data = dq.tolist()
        self.pub.publish(msg)
