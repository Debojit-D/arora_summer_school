#!/usr/bin/env python3
"""
[Student Template] Pick and Place Task — Velocity Control + Gripper Actions

Author: Debojit Das
Email: debojit.das@iitgn.ac.in

Description:
------------
This is a starter template for implementing a pick-and-place task on the HEAL robot using:
- Inverse Kinematics (PyKDL)
- Velocity-based trajectory tracking
- Grasp and release actions via ROS messages

Your task is to complete the sections marked with TODO. The robot should:
1. Move to a "home" position and release the gripper.
2. Move to a pickup position and grasp the object.
3. Move to an intermediate pose.
4. Move to a drop-off location and release the object.
5. Return to home or shutdown.

Requirements:
-------------
- ROS Noetic
- PyKDL
- addverb_cobot_msgs
- A working URDF loaded via `/robot_description`
- Your own TrajectoryPlanner class (for joint velocity trajectories)
"""

import os
import sys
import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam
from addverb_cobot_msgs.msg import GraspActionGoal, ReleaseActionGoal

# Add path for your utils module if needed
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.TRAJECTORY_PLANNERS.trajectory_planners import TrajectoryPlanner


# ------------------------ Action Publisher Utilities ------------------------

def publish_grasp_action_goal(grasp_force_value=100):
    """
    Sends a command to close the gripper using the specified force.
    """
    pub = rospy.Publisher('/robotA/grasp_action/goal', GraspActionGoal, queue_size=10)
    rospy.sleep(0.5)
    msg = GraspActionGoal()
    msg.goal.grasp_force = grasp_force_value
    pub.publish(msg)
    rospy.loginfo("Grasp command sent.")


def publish_release_action_goal():
    """
    Sends a command to open the gripper.
    """
    pub = rospy.Publisher('/robotA/release_action/goal', ReleaseActionGoal, queue_size=10)
    rospy.sleep(0.5)
    msg = ReleaseActionGoal()
    pub.publish(msg)
    rospy.loginfo("Release command sent.")


# ------------------------ Pick-and-Place Commander ------------------------

class VelocityCommander:
    def __init__(self):
        rospy.init_node("pick_and_place_commander", anonymous=True)

        # --- ROS Communication Setup ---
        self.pub = rospy.Publisher("/velocity_controller/command", Float64MultiArray, queue_size=10)
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        self.traj_planner = TrajectoryPlanner()
        self.current_joint_state = None
        self.trajectory_generated = False
        self.velocity_traj = None
        self.current_index = 0
        self.dt = 0.001  # 1ms update rate

        # --- Load Robot Model and KDL IK Chain ---
        if not rospy.has_param("robot_description"):
            rospy.logerr("robot_description not available. Is the URDF loaded?")
            exit(1)
        self.robot = URDF.from_parameter_server()
        success, tree = treeFromParam("robot_description")
        if not success:
            rospy.logerr("Failed to parse robot_description into KDL tree")
            exit(1)

        self.base_link = "base_link"
        self.tip_link = "tool_ff"
        self.chain = tree.getChain(self.base_link, self.tip_link)
        self.n_joints = self.chain.getNrOfJoints()
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)

        # ------------------ TODO 1 ------------------
        # Fill this list with poses in the following format:
        # Each pose should be a dictionary with:
        # - 'pos': kdl.Vector(x, y, z)
        # - 'quat': kdl.Rotation.Quaternion(x, y, z, w)
        # - 'action': 'open', 'close', or '' (for no action)
        # These should represent:
        # [1] Move above object and open gripper
        # [2] Move to grasp pose
        # [3] Close gripper
        # [4] Lift object
        # [5] Move to place location
        # [6] Open gripper
        # [7] Return to start
        self.poses = [
            # Example entry:
            # {
            #     'pos': kdl.Vector(0.3, 0.4, 0.5),
            #     'quat': kdl.Rotation.Quaternion(0, 0, 0, 1),
            #     'action': "open"
            # }
        ]

        self.current_pose_index = 0

    def compute_ik(self, q_init, pose_index):
        """
        Solves IK for the desired pose and returns joint angles.
        """
        target = self.poses[pose_index]
        target_frame = kdl.Frame(target['quat'], target['pos'])
        q_out = kdl.JntArray(self.n_joints)
        result = self.ik_solver.CartToJnt(q_init, target_frame, q_out)
        if result >= 0:
            return np.array([q_out[i] for i in range(self.n_joints)])
        else:
            rospy.logwarn("IK failed for pose %d", pose_index)
            return None

    def joint_state_callback(self, msg):
        """
        Callback to receive joint states and generate trajectory to current pose.
        """
        self.current_joint_state = np.array(msg.position)

        # Only generate once per pose
        if not self.trajectory_generated:
            q_init = kdl.JntArray(self.n_joints)
            for i in range(self.n_joints):
                q_init[i] = self.current_joint_state[i]

            target_joint_state = self.compute_ik(q_init, self.current_pose_index)
            if target_joint_state is None:
                return

            T = 2.5  # Duration of motion in seconds
            _, self.velocity_traj, _ = self.traj_planner.quintic_joint_trajectory(
                self.current_joint_state, target_joint_state, T, self.dt
            )
            self.trajectory_generated = True
            self.current_index = 0

    def run(self):
        """
        Main loop to run the trajectory and trigger grasp/release at poses.
        """
        rate = rospy.Rate(1.0 / self.dt)
        while not rospy.is_shutdown():
            if self.trajectory_generated and self.current_index < len(self.velocity_traj):
                msg = Float64MultiArray()
                msg.data = self.velocity_traj[self.current_index].tolist()
                self.pub.publish(msg)
                self.current_index += 1

            elif self.trajectory_generated and self.current_index >= len(self.velocity_traj):
                # Action Phase: Trigger grasp or release if defined
                action = self.poses[self.current_pose_index].get("action", "").lower().strip()
                if action == "close":
                    publish_grasp_action_goal()
                elif action == "open":
                    publish_release_action_goal()

                rospy.sleep(2.5)  # Pause before next move

                if self.current_pose_index < len(self.poses) - 1:
                    self.current_pose_index += 1
                    self.trajectory_generated = False
                else:
                    rospy.loginfo("✅ Pick-and-Place sequence completed.")
                    rospy.signal_shutdown("Done!")
            rate.sleep()


# ------------------------ Script Entry Point ------------------------

if __name__ == '__main__':
    try:
        commander = VelocityCommander()
        commander.run()
    except rospy.ROSInterruptException:
        pass
