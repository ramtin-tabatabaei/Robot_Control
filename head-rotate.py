#!/usr/bin/env python3

import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
import sys

def rotate_and_tilt_head(pan_degrees, tilt_degrees):
    global head_client
    
    # Convert degrees to radians for ROS
    pan_radians = pan_degrees * 3.141592653589793 / 180.0
    tilt_radians = tilt_degrees * 3.141592653589793 / 180.0

    # Define the goal for head movement
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_1_joint', 'head_2_joint']  # Assuming these are the correct joint names
    
    point = JointTrajectoryPoint()
    # Assign the new positions for pan (head_1_joint) and tilt (head_2_joint)
    point.positions = [pan_radians, tilt_radians]  
    point.time_from_start = rospy.Duration(3)  # Adjust the duration as needed for your setup
    trajectory.points.append(point)
    
    goal.trajectory = trajectory
    # while True:
    #     head_client.send_goal(goal)
    #     head_client.wait_for_result()
    #     rospy.sleep(0.1)
    
    head_client.send_goal(goal)
    head_client.wait_for_result()


def run():
    global head_client
    
    # Initialize the action client for controlling the head's movement
    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    if len(sys.argv) > 1:
        input_string = sys.argv[1]  # The received string (e.g., "Yaw:-30, roll:-55")
        # Splitting and extracting values
        parts = input_string.split(", ")
        yaw = int(parts[0].split(":")[1])
        roll = int(parts[1].split(":")[1])
        # print(f"Yaw: {yaw}, Roll: {roll}")
    else:
        print("No arguments received!")

    # Rotate the head 30 degrees to the right and tilt it 30 degrees down
    rotate_and_tilt_head(int(yaw), int(roll))  # Note: Assuming negative tilt means downwards

if __name__ == "__main__":
    rospy.init_node('rotate_and_tilt_head')
    run()


# import sys
# def main():
#     if len(sys.argv) > 1:
#         input_string = sys.argv[1]  # The received string (e.g., "Yaw:-30, roll:-55")
#         # Splitting and extracting values
#         parts = input_string.split(", ")
#         yaw = int(parts[0].split(":")[1])
#         roll = int(parts[1].split(":")[1])
#         print(f"Yaw: {yaw}, Roll: {roll}")
#     else:
#         print("No arguments received!")
# if __name__ == "__main__":
#     main()