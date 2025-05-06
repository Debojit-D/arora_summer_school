#!/usr/bin/env python3

"""
Trajectory Tracker using DLS Velocity Commander (Circular, Sine Wave, and Straight-Line Paths)
Author: Debojit Das (2025)

Description:
This script implements fun and fair trajectory replication tasks using Damped Least Squares (DLS) inverse kinematics. 
The robot end-effector is commanded to follow smooth, continuous paths in Cartesian space using three trajectory modes:

1. Circular Trajectory:
   - Follows a horizontal circle in the XY plane at constant Z height.
   - End-effector sweeps the loop smoothly with a constant downward orientation.

2. Sine Wave Trajectory:
   - Draws a sine wave from left to right on the XY plane.
   - Y oscillates sinusoidally as X moves between -0.3 to 0.3.
   - End-effector maintains constant orientation throughout.

3. Straight-Line Sweep Trajectory:
   - Moves back and forth along a straight line on the X-axis.
   - Y and Z remain fixed.
   - Initial movement begins from center to one end, then sweeps across to the opposite end.

Features:
- Smooth, continuous Cartesian trajectory tracking via velocity-based DLS IK.
- Live 100 Hz control loop.
- Safe shutdown: stops robot by publishing zero velocity on Ctrl+C.
- 3D trajectory visualization on shutdown for analysis and debugging.
"""


# Import essential libraries
import rospy                                     # ROS Python client library
import numpy as np                               # For mathematical operations
from dls_velocity_commander import DLSVelocityCommander  # Our custom DLS IK controller
from tf.transformations import quaternion_from_euler    # Converts Euler angles to quaternions
import PyKDL as kdl                              # Kinematics and dynamics library
import matplotlib.pyplot as plt                  # For plotting the robot's path in 3D
from std_msgs.msg import Float64MultiArray       # ROS message type for publishing joint velocities

# ------------------ Trajectories ------------------

# 1. Circular path in XY plane at fixed Z height.
def circular_trajectory(t, radius=0.25, center=[0.0, 0.368, 0.553], omega=0.25):
    """
    Generate a circular trajectory in the XY-plane.

    Parameters:
    - radius: radius of the circle (meters)
    - center: [x, y, z] center of the circle
    - omega: angular speed (radians/second)

    Returns:
    - pos: 3D position [x, y, z]
    - quat: constant orientation as quaternion (pointing downward)
    """
    x = center[0] + radius * np.cos(omega * t)
    y = center[1] + radius * np.sin(omega * t)
    z = center[2]
    pos = [x, y, z]

    # End-effector Z-axis points down (180° roll)
    quat = quaternion_from_euler(np.pi, 0, 0)
    return pos, quat

# 2. Sine wave in XY-plane from x = -x_range to +x_range
def sine_wave_trajectory(t, x_range=0.3, y_center=0.368, y_amplitude=0.1, z_height=0.553, period=8.0):
    """
    Generate a sine wave path along X, with Y oscillating.

    Parameters:
    - x_range: half-width of the wave span in X
    - y_center: central Y position
    - y_amplitude: amplitude of sine wave in Y
    - z_height: constant Z position
    - period: time (s) to complete a full left-right sweep

    Returns:
    - pos: [x, y, z] along a sine wave
    - quat: constant downward-facing orientation
    """
    sweep_phase = np.sin(2 * np.pi * t / period)
    x = sweep_phase * x_range
    y = y_center + y_amplitude * np.sin(np.pi * x / x_range)
    z = z_height
    pos = [x, y, z]
    quat = quaternion_from_euler(np.pi, 0, 0)
    return pos, quat

# 3. Straight-line trajectory (left to right and back)
def straight_line_trajectory(t, x_min=-0.4, x_max=0.8, y_val=0.368, z_val=0.553, period=20.0):
    """
    Generate a to-and-fro straight-line motion between x_min and x_max.

    Parameters:
    - x_min/x_max: range to sweep in X
    - y_val/z_val: fixed Y and Z values
    - period: duration of full sweep (left -> right -> left)

    Returns:
    - pos: [x, y, z] moving linearly over time
    - quat: fixed downward orientation
    """
    phase = np.sin(2 * np.pi * t / period)       # Smooth oscillation between -1 and 1
    x = (x_max + x_min)/2 + (x_max - x_min)/2 * phase
    pos = [x, y_val, z_val]
    quat = quaternion_from_euler(np.pi, 0, 0)
    return pos, quat


# ------------------ Plotting ------------------

def plot_trajectory(traj, title="End-Effector Trajectory"):
    """
    Plot the trajectory traced by the robot's end-effector in 3D.

    Parameters:
    - traj: list of [x, y, z] positions
    - title: title of the plot
    """
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

# ------------------ Main Execution ------------------

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("trajectory_tracking_dls")

    # Start from initial position generated by any trajectory (e.g., sine wave)
    init_pos, init_quat = sine_wave_trajectory(0.0)

    # Create the IK commander using Damped Least Squares method
    commander = DLSVelocityCommander(target_pos=init_pos, target_quat=init_quat)

    # Publisher to manually stop the robot with zero velocity
    pub_zero = rospy.Publisher("/velocity_controller/command", Float64MultiArray, queue_size=10)

    # 100 Hz control loop
    rate = rospy.Rate(100)
    t_start = rospy.get_time()
    trajectory_log = []  # Store EE positions for plotting

    # Define what to do on shutdown (Ctrl+C)
    def shutdown_hook():
        rospy.loginfo("Shutting down. Sending zero velocity command.")
        
        # Stop all joints by publishing zero velocity
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0] * commander.n_joints
        pub_zero.publish(zero_msg)

        # Plot the tracked trajectory in 3D
        plot_trajectory(trajectory_log)

    # Register shutdown hook
    rospy.on_shutdown(shutdown_hook)

    # ----------- CONTROL LOOP -----------
    try:
        while not rospy.is_shutdown():
            # Get current elapsed time
            t = rospy.get_time() - t_start

            # Choose any of the 3 available trajectory generators:
            #pos, quat = circular_trajectory(t)
            pos, quat = sine_wave_trajectory(t)
            #pos, quat = straight_line_trajectory(t)  # Active line tracking

            # Update the target pose in the IK solver
            commander.target_pos = kdl.Vector(*pos)
            commander.target_quat = kdl.Rotation.Quaternion(*quat)
            commander.target_frame = kdl.Frame(commander.target_quat, commander.target_pos)

            # Perform one IK step and publish joint velocities
            commander.run_once()

            # Log the end-effector position for plotting
            trajectory_log.append(pos)

            # Sleep to maintain control rate
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
