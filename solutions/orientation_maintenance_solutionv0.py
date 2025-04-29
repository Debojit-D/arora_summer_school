#!/usr/bin/env python3
"""
Arc Follower using DLS Velocity Commander (Vertical X-Z Arc, Full Sweep)
Author: Debojit Das (2025)

Description:
- Start from center, sweep +30°, then -30°, and return to center.
- Arc center is the current end-effector position.
- Radius = z-coordinate of center.
"""

import os
import sys
import rospy
import numpy as np
import PyKDL as kdl
from geometry_msgs.msg import PoseStamped

# Import DLSVelocityCommander
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from dls_velocity_commander import DLSVelocityCommander

# --------------- User Configurable Section ---------------
num_points = 50                     # Number of waypoints per segment
num_points = 50                     # Number of waypoints per segment
arc_max_angle_deg = 60                # Maximum arc angle in degrees (+30° and -30°)
goal_reach_tolerance = 0.01           # 1 cm tolerance
target_quat = [0.0, 0.0, 0.0, 1.0]    # Maintain neutral orientation
# ----------------------------------------------------------

# Global storage for live EE position
current_ee_position = None

def end_effector_pose_callback(msg):
    """Callback to update live end-effector position."""
    global current_ee_position
    current_ee_position = np.array([
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z
    ])

def generate_arc_waypoints(center, radius):
    """Generate full arc waypoints: center -> +30° -> center -> -30° -> center."""
    # Define sequences
    angles_pos = np.linspace(0, np.deg2rad(arc_max_angle_deg), num_points)
    angles_neg = np.linspace(np.deg2rad(arc_max_angle_deg), np.deg2rad(-arc_max_angle_deg), 2 * num_points)
    angles_return = np.linspace(np.deg2rad(-arc_max_angle_deg), 0, num_points)

    angles_total = np.concatenate([angles_pos, angles_neg, angles_return])

    waypoints = []
    for theta in angles_total:
        x = center[0] + radius * np.sin(theta)
        y = center[1]                           # Y remains constant
        z = center[2] - radius * (1 - np.cos(theta))
        waypoints.append((x, y, z))

    return np.array(waypoints)

def main():
    global current_ee_position

    rospy.init_node("arc_follower_dls_vertical_full", anonymous=True)

    # Subscribe to end effector pose
    rospy.Subscriber("/end_effector_pose", PoseStamped, end_effector_pose_callback)

    print("📡 Mr. Kala Jadu says: Waiting for initial End Effector Pose... ⏳")

    # Wait until first pose received
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown() and current_ee_position is None:
        rate.sleep()

    # Capture the starting center
    center = np.array(current_ee_position)
    radius = center[2]  # Use Z coordinate as radius

    print(f"✅ Mr. Kala Jadu says: Starting center recorded: {center}")

    # Generate full arc waypoints
    arc_waypoints = generate_arc_waypoints(center, radius)

    # Initialize DLS controller with first point
    commander = DLSVelocityCommander(
        target_pos=arc_waypoints[0],
        target_quat=target_quat
    )

    print("🧙‍♂️ Mr. Kala Jadu says: Starting full arc journey! 🚀")

    rate = rospy.Rate(100)  # 100 Hz

    idx = 0
    total_points = len(arc_waypoints)

    while not rospy.is_shutdown() and idx < total_points:
        if current_ee_position is None:
            rate.sleep()
            continue

        target_pos = arc_waypoints[idx]

        # Check if close enough to current target
        distance = np.linalg.norm(current_ee_position - target_pos)

        if distance < goal_reach_tolerance:
            print(f"🎯 Reached point {idx+1}/{total_points} ➡️ Moving to next point!")
            idx += 1
            if idx < total_points:
                # Update target in commander
                commander.target_pos = kdl.Vector(*arc_waypoints[idx])
                commander.target_frame = kdl.Frame(commander.target_quat, commander.target_pos)

        commander.run_once()  # Only one iteration per loop
        rate.sleep()

    print("🎉 Mr. Kala Jadu says: Full Arc Challenge completed! 🌟")

if __name__ == "__main__":
    main()
