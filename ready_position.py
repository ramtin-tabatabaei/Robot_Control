#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64

# Global variables
current_joint_positions = {}
arm_client = None
head_client = None

def joint_state_callback(msg):
    """
    Callback function for /joint_states to update current joint positions.
    """
    global current_joint_positions
    for i, name in enumerate(msg.name):
        current_joint_positions[name] = msg.position[i]

def get_current_joint_positions(joint_names):
    """
    Get the current joint positions for the specified joint names.
    """
    rospy.sleep(0.5)  # Wait to ensure callback has populated the data
    return [current_joint_positions.get(joint, 0.0) for joint in joint_names]

def sort_assemb_start():
    """
    Main function to start the sorting assembly position.
    """
    global arm_client, head_client

    # Initialize action clients
    arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    # rospy.loginfo("Arm server connected.")

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()
    # rospy.loginfo("Head server connected.")

    # Rotate and tilt head
    rotate_and_tilt_head(0, -10, 2)  # Rotate head 0 degrees, tilt down 10 degrees

    # Get current joint positions
    joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
                   'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
    joint_angles_start = get_current_joint_positions(joint_names)

    # Define target positions for the arm
    joint_angles_end = [0.07, 0.47, -1.53, 1.74, 0.37, -1.37, 0.28]
    move_arm(joint_angles_start, joint_angles_end, t=6)

    # rospy.loginfo("Finished gesture.")

def rotate_and_tilt_head(pan_degrees, tilt_degrees, duration):
    """
    Rotate and tilt the robot's head.
    """
    global head_client

    # Convert degrees to radians
    pan_radians = np.radians(pan_degrees)
    tilt_radians = np.radians(tilt_degrees)

    # Define the goal for head movement
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_1_joint', 'head_2_joint']  # Adjust for your robot

    # Set positions and duration
    point = JointTrajectoryPoint()
    point.positions = [pan_radians, tilt_radians]
    point.time_from_start = rospy.Duration(duration)
    trajectory.points.append(point)
    
    goal.trajectory = trajectory

    # Send the goal
    head_client.send_goal(goal)
    head_client.wait_for_result()

def move_arm(joint_angles_start, joint_angles_end, t, steps=50):
    """
    Move the robot arm with cosine-interpolated smooth motion.
    """
    global arm_client

    # Define the goal
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = [
        'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
        'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
    ]

    # Generate interpolated points
    times = np.linspace(0, t, steps)
    for i in range(steps):
        alpha = (1 - np.cos(np.pi * (i / (steps - 1)))) / 2  # Cosine interpolation factor
        interpolated_positions = [
            start + alpha * (end - start)
            for start, end in zip(joint_angles_start, joint_angles_end)
        ]
        point = JointTrajectoryPoint()
        point.positions = interpolated_positions
        point.time_from_start = rospy.Duration(times[i])
        trajectory.points.append(point)

    # Set the trajectory in the goal
    goal.trajectory = trajectory

    # Send the goal and wait for the result
    # rospy.loginfo("Sending goal for arm movement with smooth interpolation...")
    arm_client.send_goal(goal)
    # if arm_client.wait_for_result(rospy.Duration(t + 1)):
    #     rospy.loginfo("Arm completed successfully with smooth motion.")
    # else:
    #     rospy.loginfo("Arm did not complete before the timeout.")

if __name__ == '__main__':
    rospy.init_node('start_position')

    # Subscribe to /joint_states
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    # Start the sorting assembly
    sort_assemb_start()

    # rospy.loginfo("Finished start_position.")
    print("1")
