#!/usr/bin/env python3
"""
Arc Follower using DLS Velocity Commander (Vertical X-Z Arc, Full Sweep, Fully Smooth Orientation)
Author: Debojit Das (2025)

Description:
- Start from center, sweep +30°, then -30°, then return to center.
- Center is initial EE position.
- Radius = Z coordinate of center.
- End-effector Z-axis always points TOWARDS the center.
- End-effector X-axis preserves initial X-axis as much as possible (Gram-Schmidt projection).
"""

import os
import sys
import rospy
import numpy as np
import PyKDL as kdl
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped

# Import DLSVelocityCommander
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from dls_velocity_commander import DLSVelocityCommander

# --------------- Config Section ---------------
num_points = 50                    # Waypoints per arc segment
arc_max_angle_deg = 30              # Max arc sweep +30° and -30°
goal_reach_tolerance = 0.01          # 1cm
# ------------------------------------------------

current_ee_position = None
current_ee_quaternion = None

def end_effector_pose_callback(msg):
    global current_ee_position, current_ee_quaternion
    current_ee_position = np.array([
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z
    ])
    current_ee_quaternion = np.array([
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    ])

def generate_arc_waypoints(center, radius):
    angles_pos = np.linspace(0, np.deg2rad(arc_max_angle_deg), num_points)
    angles_neg = np.linspace(np.deg2rad(arc_max_angle_deg), np.deg2rad(-arc_max_angle_deg), 2 * num_points)
    angles_return = np.linspace(np.deg2rad(-arc_max_angle_deg), 0, num_points)
    angles_total = np.concatenate([angles_pos, angles_neg, angles_return])

    waypoints = []
    for theta in angles_total:
        x = center[0] + radius * np.sin(theta)
        y = center[1]
        z = center[2] - radius * (1 - np.cos(theta))
        waypoints.append((x, y, z))
    return np.array(waypoints)

def project_onto_plane(v, n):
    """Project vector v onto plane orthogonal to n."""
    n = n / np.linalg.norm(n)
    return v - np.dot(v, n) * n

def compute_orientation(center, ee_position, x_axis_start):
    """
    Compute rotation matrix where:
    - Z-axis points towards center.
    - X-axis is original starting X projected onto plane orthogonal to Z.
    """
    z_axis = center - ee_position
    z_axis /= np.linalg.norm(z_axis)

    # Project starting x_axis onto plane orthogonal to z_axis
    x_axis = project_onto_plane(x_axis_start, z_axis)
    x_axis /= np.linalg.norm(x_axis)

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    R = np.column_stack((x_axis, y_axis, z_axis))

    quat = tf_trans.quaternion_from_matrix(
        np.vstack((np.hstack((R, np.array([[0],[0],[0]]))), np.array([0,0,0,1])))
    )
    return quat

def main():
    global current_ee_position, current_ee_quaternion

    rospy.init_node("arc_follower_dls_vertical_final", anonymous=True)

    rospy.Subscriber("/end_effector_pose", PoseStamped, end_effector_pose_callback)

    print("📡 Mr. Kala Jadu says: Waiting for initial EE pose and orientation... ⏳")

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown() and (current_ee_position is None or current_ee_quaternion is None):
        rate.sleep()

    center = np.array(current_ee_position)
    starting_quat = np.array(current_ee_quaternion)
    radius = center[2]

    print(f"✅ Mr. Kala Jadu says: Starting center recorded: {center}")

    arc_waypoints = generate_arc_waypoints(center, radius)

    # Extract starting X-axis from initial orientation
    R_start = tf_trans.quaternion_matrix(starting_quat)[:3, :3]
    x_axis_start = R_start[:, 0]  # First column

    # Compute initial target orientation
    initial_quat = compute_orientation(center, arc_waypoints[0], x_axis_start)

    commander = DLSVelocityCommander(
        target_pos=arc_waypoints[0],
        target_quat=initial_quat
    )

    print("🧙‍♂️ Mr. Kala Jadu says: Starting magical full arc journey! 🚀")

    idx = 0
    total_points = len(arc_waypoints)
    rate = rospy.Rate(100)  # 100Hz

    while not rospy.is_shutdown() and idx < total_points:
        if current_ee_position is None:
            rate.sleep()
            continue

        target_pos = arc_waypoints[idx]
        distance = np.linalg.norm(current_ee_position - target_pos)

        if distance < goal_reach_tolerance:
            print(f"🎯 Reached point {idx+1}/{total_points} ➡️ Moving to next!")
            idx += 1
            if idx < total_points:
                commander.target_pos = kdl.Vector(*arc_waypoints[idx])

                dynamic_quat = compute_orientation(center, arc_waypoints[idx], x_axis_start)
                commander.target_quat = kdl.Rotation.Quaternion(*dynamic_quat)
                commander.target_frame = kdl.Frame(commander.target_quat, commander.target_pos)

        commander.run_once()
        rate.sleep()

    print("🎉 Mr. Kala Jadu says: Full Arc Challenge completed successfully! 🌟")

if __name__ == "__main__":
    main()
