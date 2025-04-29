#!/usr/bin/env python3
"""
Solution Script: Relative Angle Reaching Task
Author: Debojit Das (2025)

Description:
- Generate random target joint angles for 6-DOF robot (custom range per joint).
- User manually moves the robot to the target configuration in free-drive mode.
- After the user confirms, evaluate the final joint angles against the target.
- Provide scoring and feedback.
- Optionally, move the robot automatically to the target angles.
- Mr. Kala Jadu guides the user with fun, engaging messages and emojis.
"""

import numpy as np
import time
import rospy
from sensor_msgs.msg import JointState

# --------------- User Configurable Section ---------------
NUM_JOINTS = 6  # Assuming 6-DOF robot

# Define individual angle limits for each joint (degrees)
JOINT_LIMITS_DEG = [
    (-90, 90),    # Joint 1 limits [Anti-Clockwise Positive]
    (-45, 45),    # Joint 2 limits [Anti-Clockwise Positive]
    (-30, 60),    # Joint 3 limits [Anti-clockeise Positive]
    (-90, 90),    # Joint 4 limits 
    (0, 90),      # Joint 5 limits
    (-60, 60)     # Joint 6 limits
]

ANGLE_TOLERANCE = 5  # Allowed tolerance in degrees for success
WAIT_AFTER_MOVEMENT = 2  # Wait seconds after robot moves
# ----------------------------------------------------------

# Global storage for received joint angles
current_joint_angles_rad = None

def joint_states_callback(msg):
    """Callback to store latest joint angles."""
    global current_joint_angles_rad
    current_joint_angles_rad = np.array(msg.position)  # msg.position is in radians

def get_current_joint_angles():
    """
    Reads the latest joint angles from the /joint_states topic.
    Returns:
        - joint angles in degrees (numpy array)
    """
    global current_joint_angles_rad

    # Initialize ROS node if not already initialized
    if not rospy.core.is_initialized():
        rospy.init_node('relative_angle_reader', anonymous=True)

    # Subscribe once if not already
    subscriber = rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    print("📡 Mr. Kala Jadu says: Listening to /joint_states for latest robot angles... ⏳")
    rate = rospy.Rate(10)  # 10 Hz

    # Wait until we receive at least one valid message
    timeout_sec = 5
    start_time = rospy.get_time()
    while current_joint_angles_rad is None:
        if rospy.get_time() - start_time > timeout_sec:
            print("⏰ Timeout while waiting for /joint_states! Returning zeros.")
            return np.zeros(NUM_JOINTS)
        rate.sleep()

    # After message received, shutdown subscriber cleanly
    rospy.signal_shutdown('Received joint angles, shutting down listener.')

    # Convert radians to degrees
    current_joint_angles_deg = np.degrees(current_joint_angles_rad)

    # Return only first 6 joints if more are present
    return np.round(current_joint_angles_deg[:NUM_JOINTS], 2)

def move_robot_to_joint_angles(target_angles_deg):
    """
    Mock function to move robot to target joint angles.
    Replace with actual function commanding the robot.
    """
    print(f"🤖 Moving robot to target angles... Please wait!")
    time.sleep(WAIT_AFTER_MOVEMENT)  # Simulate movement delay
    print(f"✅ Robot reached target angles!")

# --------------- Main Task Execution ---------------
def generate_target_angles():
    """Generate random integer target angles based on per-joint limits."""
    target_angles = []
    for idx, (low, high) in enumerate(JOINT_LIMITS_DEG):
        angle = np.random.randint(low, high + 1)  # Integer in [low, high]
        target_angles.append(angle)
    return np.array(target_angles)

def evaluate_user_performance(target_angles, achieved_angles):
    """Evaluate how close the achieved angles are to the target angles."""
    signed_errors = achieved_angles - target_angles  # Signed difference
    abs_errors = np.abs(signed_errors)  # Magnitude for tolerance checking
    success = np.all(abs_errors <= ANGLE_TOLERANCE)
    return success, signed_errors

def print_target_angles(target_angles):
    """Print the target angles nicely."""
    print("\n🎯 Mr. Kala Jadu says: Here are your magical target angles!")
    for idx, angle in enumerate(target_angles):
        print(f"🔹 Joint {idx+1}: {angle} degrees")

def print_evaluation_result(success, signed_errors):
    """Print the evaluation result showing both magnitude and direction."""
    if success:
        print("\n🏆 Mr. Kala Jadu says: Bravo! You nailed it! 🎉")
    else:
        print("\n❗ Mr. Kala Jadu says: Not bad, but needs more magic! ✨")

    print("\n📈 Here’s your performance per joint:")
    for idx, err in enumerate(signed_errors):
        direction = "positive overshoot ➡️" if err > 0 else "negative undershoot ⬅️" if err < 0 else "perfect 🎯"
        print(f"   - Joint {idx+1}: Error = {abs(err):.2f}° ({direction})")

def main():
    print("\n🧙‍♂️ Welcome, brave warrior! Mr. Kala Jadu summons you for the Angle Reaching Challenge! 🧙‍♂️\n")
    
    # 1. Generate Target Angles
    target_angles = generate_target_angles()
    print_target_angles(target_angles)
    
    # 2. User Free Drives the Robot
    input("\n🖐️ Mr. Kala Jadu says: Now manually move the robot to these angles. Press [Enter] when you are DONE...")

    # 3. Read Current Angles
    print("\n📡 Reading current robot joint angles...")
    achieved_angles = get_current_joint_angles()
    
    print("\n🤔 Mr. Kala Jadu says: Evaluating your performance...")
    time.sleep(1)
    
    # 4. Evaluate User's Attempt
    success, errors = evaluate_user_performance(target_angles, achieved_angles)
    print_evaluation_result(success, errors)
    
    # 5. Offer Robot Auto-Move
    choice = input("\n🤖 Mr. Kala Jadu asks: Want me to move the robot to the correct angles? (yes/no): ").strip().lower()
    if choice in ['yes', 'y']:
        move_robot_to_joint_angles(target_angles)
    else:
        print("\n🛑 Mr. Kala Jadu says: No worries! Practice makes perfect. Keep trying! 💪")

    print("\n🎉 Task completed! Mr. Kala Jadu waves goodbye! 👋\n")

# --------------- Run the script ---------------
if __name__ == "__main__":
    main()
