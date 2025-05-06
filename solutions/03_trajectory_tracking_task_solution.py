#!/usr/bin/env python3

"""
Trajectory Tracker using DLS Velocity Commander (Circular, Sine Wave, and Straight-Line Paths)
Author: Debojit Das (2025)
Email: debojit.das@iitgn.ac.in

Overview:
---------
This script enables trajectory replication for the HEAL robot using velocity-based Damped Least Squares (DLS) inverse kinematics.

Supported Trajectories:
------------------------
1. Circular: Tangential orientation maintained for tasks like holding a ring.
2. Sine Wave: Now **bidirectional** (returns via the same path).
3. Straight Line: Left–right sweep in Cartesian X.
4. Custom: Easily plug in your own function.

Safety Note:
------------
⚠️ Angular velocity control is **disabled** by default in DLS to prevent cable winding.
You may enable it in `compute_dls_ik()` by un-commenting the angular velocity line.
"""

# ------------------ Imports ------------------

import rospy
import numpy as np
import PyKDL as kdl
import matplotlib.pyplot as plt
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray
from dls_velocity_commander import DLSVelocityCommander

# ------------------ Trajectories ------------------

def circular_trajectory(t, radius=0.25, center=[0.0, 0.368, 0.553], omega=0.25):
    angle = omega * t
    x = center[0] + radius * np.cos(angle)
    y = center[1] + radius * np.sin(angle)
    z = center[2]
    pos = [x, y, z]

    dx = -radius * np.sin(angle)
    dy = radius * np.cos(angle)
    dz = 0.0
    tangent = np.array([dx, dy, dz]) / np.linalg.norm([dx, dy, dz])
    z_axis = np.array([0, 0, 1])
    y_axis = np.cross(z_axis, tangent)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    R = np.column_stack((x_axis, y_axis, z_axis))
    rot = kdl.Rotation(R[0, 0], R[0, 1], R[0, 2],
                       R[1, 0], R[1, 1], R[1, 2],
                       R[2, 0], R[2, 1], R[2, 2])
    quat = rot.GetQuaternion()
    return pos, quat

def sine_wave_bidirectional(t, period=8.0, x_range=0.3, y_center=0.368, y_amplitude=0.1, z_height=0.553):
    full_period = 2 * period
    local_t = t % full_period
    direction = 1 if local_t < period else -1
    phase = direction * np.sin(np.pi * (local_t % period) / period)
    x = phase * x_range
    y = y_center + y_amplitude * np.sin(np.pi * x / x_range)
    z = z_height
    pos = [x, y, z]
    quat = quaternion_from_euler(np.pi, 0, 0)
    return pos, quat

def straight_line_trajectory(t, x_min=-0.4, x_max=0.4, y_val=0.368, z_val=0.553, period=20.0):
    phase = np.sin(2 * np.pi * t / period)
    x = (x_max + x_min) / 2 + (x_max - x_min) / 2 * phase
    pos = [x, y_val, z_val]
    quat = quaternion_from_euler(np.pi, 0, 0)
    return pos, quat

def custom_trajectory(t):
    """
    Add your own custom trajectory here.
    Replace `pos` and `quat` with desired values.
    """
    pos = [0.2, 0.3, 0.55]  # Example static position
    quat = quaternion_from_euler(np.pi, 0, 0)  # Downward-facing
    return pos, quat

# ------------------ Plotting ------------------

def plot_trajectory(traj, title="End-Effector Trajectory"):
    traj = np.array(traj)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], label='EE Path', lw=2)
    ax.scatter([0], [0], [0], color='r', label='Origin', s=50)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()

# ------------------ Main ------------------

if __name__ == '__main__':
    init_pos, init_quat = sine_wave_bidirectional(0.0)
    commander = DLSVelocityCommander(target_pos=init_pos, target_quat=init_quat)
    pub_zero = rospy.Publisher("/velocity_controller/command", Float64MultiArray, queue_size=10)
    trajectory_log = []

    def shutdown_hook():
        rospy.loginfo("Shutting down. Sending zero velocity command.")
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0] * commander.n_joints
        pub_zero.publish(zero_msg)
        plot_trajectory(trajectory_log)

    rospy.on_shutdown(shutdown_hook)

    rate = rospy.Rate(100)
    t_start = rospy.get_time()

    try:
        while not rospy.is_shutdown():
            t = rospy.get_time() - t_start

            # Select one trajectory:
            #pos, quat = circular_trajectory(t)
            #pos, quat = sine_wave_bidirectional(t)
            pos, quat = straight_line_trajectory(t)
            #pos, quat = custom_trajectory(t)

            commander.target_pos = kdl.Vector(*pos)
            commander.target_quat = kdl.Rotation.Quaternion(*quat)
            commander.target_frame = kdl.Frame(commander.target_quat, commander.target_pos)
            commander.run_once()
            trajectory_log.append(pos)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
