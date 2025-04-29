#!/usr/bin/env python3
import os
import sys
import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

class DLSVelocityCommander:
    def __init__(self, target_pos, target_quat, custom_ds=None):
        """
        Parameters:
        - target_pos: List of 3 floats [x, y, z]
        - target_quat: List of 4 floats [qx, qy, qz, qw]
        - custom_ds: Optional callable f(x: np.ndarray) -> np.ndarray that returns desired velocity
        """
        #rospy.init_node("dls_velocity_commander", anonymous=True)

        self.pub = rospy.Publisher("/velocity_controller/command", Float64MultiArray, queue_size=10)
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        self.damping = 0.1
        self.dt = 0.1
        self.max_cartesian_vel = 0.035  # Linear velocity cap (m/s)
        self.max_angular_vel = 0.05      # Angular velocity cap (rad/s)
        self.custom_ds = custom_ds

        # Setup KDL
        if not rospy.has_param("robot_description"):
            rospy.logerr("Parameter 'robot_description' not set")
            exit(1)
        self.robot = URDF.from_parameter_server()
        success, tree = treeFromParam("robot_description")
        if not success:
            rospy.logerr("Failed to construct KDL tree from URDF")
            exit(1)

        self.base_link = "base_link"
        self.tip_link = "tool_ff"
        self.chain = tree.getChain(self.base_link, self.tip_link)
        self.n_joints = self.chain.getNrOfJoints()
        rospy.loginfo("KDL chain successfully created with %d joints", self.n_joints)

        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

        # Set user-defined target frame
        self.target_pos = kdl.Vector(*target_pos)
        self.target_quat = kdl.Rotation.Quaternion(*target_quat)
        self.target_frame = kdl.Frame(self.target_quat, self.target_pos)

        self.current_joint_state = None

    def compute_dls_ik(self, q_current):
        current_frame = kdl.Frame()
        self.fk_solver.JntToCart(q_current, current_frame)

        # --- Linear velocity computation ---
        current_pos = np.array([current_frame.p.x(), current_frame.p.y(), current_frame.p.z()])
        if self.custom_ds:
            linear_error = self.custom_ds(current_pos)
        else:
            pos_error = self.target_frame.p - current_frame.p
            linear_error = np.array([pos_error.x(), pos_error.y(), pos_error.z()])

        norm_lin = np.linalg.norm(linear_error)
        if norm_lin > self.max_cartesian_vel:
            linear_error = (linear_error / norm_lin) * self.max_cartesian_vel

        # --- Angular velocity computation ---
        R_current = current_frame.M
        R_target = self.target_frame.M
        R_err = R_target * R_current.Inverse()

        angle, axis = R_err.GetRotAngle()
        axis_np = np.array([axis.x(), axis.y(), axis.z()])
        angular_error = axis_np * angle

        norm_ang = np.linalg.norm(angular_error)
        if norm_ang > self.max_angular_vel:
            angular_error = (angular_error / norm_ang) * self.max_angular_vel

        # Combine into 6D error vector
        error_vec = np.concatenate((linear_error, angular_error))

        # --- Jacobian and DLS pseudoinverse ---
        jacobian = kdl.Jacobian(self.n_joints)
        self.jac_solver.JntToJac(q_current, jacobian)
        J = np.zeros((6, self.n_joints))
        for i in range(6):
            for j in range(self.n_joints):
                J[i, j] = jacobian[i, j]

        lambda_sq = self.damping ** 2
        damped_term = J @ J.T + lambda_sq * np.eye(6)

        try:
            inv_term = np.linalg.inv(damped_term)
        except np.linalg.LinAlgError:
            rospy.logerr("Damped term inversion failed!")
            return np.zeros(self.n_joints)

        J_dls = J.T @ inv_term
        dq = J_dls @ error_vec
        return dq

    def joint_state_callback(self, msg):
        self.current_joint_state = np.array(msg.position)

    def run_once(self):
        """Run one control step without infinite looping."""
        if self.current_joint_state is None:
            return

        q_current = kdl.JntArray(self.n_joints)
        for i in range(self.n_joints):
            q_current[i] = self.current_joint_state[i]

        dq = self.compute_dls_ik(q_current)

        msg = Float64MultiArray()
        msg.data = dq.tolist()
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(1.0 / self.dt)
        while not rospy.is_shutdown():
            if self.current_joint_state is None:
                rate.sleep()
                continue

            q_current = kdl.JntArray(self.n_joints)
            for i in range(self.n_joints):
                q_current[i] = self.current_joint_state[i]

            dq = self.compute_dls_ik(q_current)

            msg = Float64MultiArray()
            msg.data = dq.tolist()
            self.pub.publish(msg)

            rate.sleep()
