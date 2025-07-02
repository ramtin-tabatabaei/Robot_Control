#!/usr/bin/env python3

import rospy
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from math import pi, inf
import csv
import sys
from pal_interaction_msgs.msg import TtsAction, TtsGoal

from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations  # For converting between different representations
from std_msgs.msg import Float32MultiArray

import numpy as np


################# COSINE INTERPOLATION HELPERS #################
def cosine_interp_list(a, b, num_points):
    lst = []
    for i in range(len(a)):
        lst.append(cosine_interp_vals(a[i], b[i], num_points))
    return lst

def cosine_interp_vals(a, b, num_points):
    t = np.linspace(0, 1, num_points)
    # Transform t to the cosine space
    t = (1 - np.cos(t * np.pi)) / 2
    return [(1-tt) * a + tt * b for tt in t]



# Subscriber callback for updating current joint positions
def joint_state_callback(msg):
    global current_joint_positions, current_gripper_position, current_head_position
    try:
        for i, name in enumerate(joint_names):
            index = msg.name.index(name)
            current_joint_positions[i] = msg.position[index]
        gripper_index = msg.name.index("gripper_left_finger_joint")
        current_gripper_position[0] = msg.position[gripper_index]*2
        current_gripper_position[1] = msg.position[gripper_index]*2
        head_1_index = msg.name.index("head_1_joint")
        head_2_index = msg.name.index("head_2_joint")
        current_head_position[0] = msg.position[head_1_index]
        current_head_position[1] = msg.position[head_2_index]
    except Exception as e:
        rospy.logerr(f"Error in joint_state_callback: {e}")

# Your other functions (update_gripper_position, update_head_position, apply_joint_positions, get_current_end_effector_pose) remain the same


def update_gripper_position(increment):
    global current_gripper_position

    # Update the current position based on the increment/decrement value
    # new_position = [pos + increment for pos in current_gripper_position]
    new_position = [increment, increment]

    # Ensure the new position is within the allowable range
    # Assuming the gripper range is between 0 (fully closed) and 0.04 (fully open)
    new_position = [max(0.02, min(1, pos)) for pos in new_position]

    # Update the global variable
    current_gripper_position = new_position

    # Create and send the new goal to the action server
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    point = JointTrajectoryPoint()
    point.positions = current_gripper_position
    point.time_from_start = rospy.Duration(0.5)
    trajectory.points.append(point)
    goal.trajectory = trajectory
    
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


def update_head_position(pan_degrees, tilt_degrees, duration):
    global current_head_position
    
    # Convert degrees to radians for ROS
    pan_radians = pan_degrees * 3.141592653589793 / 180.0
    tilt_radians = tilt_degrees * 3.141592653589793 / 180.0

    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_1_joint', 'head_2_joint']
    point = JointTrajectoryPoint()
    point.positions =  [pan_radians, tilt_radians] 
    point.time_from_start = rospy.Duration(duration)
    trajectory.points.append(point)
    goal.trajectory = trajectory
    
    head_client.send_goal(goal)
    head_client.wait_for_result()


def apply_joint_positions_with_interp(joint_position_dict, duration):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    #traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names

    start_joint_list = [current_joint_positions[i] for i in range(number_of_joints)]
    end_joint_list = [0] * len(joint_names)
    for i, name in enumerate(joint_names):
        end_joint_list[i] = joint_position_dict[name]

    num_interp = 100
    interp_lists = cosine_interp_list(start_joint_list, end_joint_list, num_interp)

    # add all joint lists
    for i in range(num_interp):

        this_joint_list = []
        for joint_index in range(len(interp_lists)):
            this_joint_list.append(interp_lists[joint_index][i])
    
        point = JointTrajectoryPoint()
        
        point.positions = this_joint_list
        # print(duration/(num_interp-1))
        point.time_from_start = rospy.Duration(i*(duration/(num_interp-1)))  # Adjust based on your requirements
        traj_msg.points.append(point)

    # print("message", traj_msg)
    
    # Publish the message
    arm_pub.publish(traj_msg)


def apply_joint_positions(joint_position_dict, duration):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    #traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names
    
    point = JointTrajectoryPoint()

    all_position = [0] * len(joint_names)

    for i, name in enumerate(joint_names):
        all_position[i] = joint_position_dict[name]
    
    point.positions = all_position
    point.time_from_start = rospy.Duration(duration)  # Adjust based on your requirements
    traj_msg.points.append(point)

    # print("message", traj_msg)
    
    # Publish the message
    arm_pub.publish(traj_msg)

# Function to get the current pose of the end-effector
def get_current_end_effector_pose():
    current_pose = Frame()
    fk_solver.JntToCart(current_joint_positions, current_pose)
    return current_pose



def move_to_goal_position(goal_position, goal_orientation, duration):
    global desired_frame, current_joint_positions, ik_solver_pos
    # Set desired frame based on the goal position and orientation
    desired_position = Vector(*goal_position)
    desired_orientation = Rotation.RPY(*goal_orientation)
    desired_frame = Frame(desired_orientation, desired_position)
    
    # Use IK to calculate desired joint positions
    ik_result = ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
    if ik_result >= 0:  # If IK solution is found
        joint_positions_dict = {joint_names[i]: desired_joint_positions[i] for i in range(number_of_joints)}
        # apply_joint_positions(joint_positions_dict, duration)
        apply_joint_positions_with_interp(joint_positions_dict, duration)
    else:
        rospy.logerr("Failed to find an IK solution for the desired position.")

def yaw_converter(yaw):
    if yaw > pi/4 and yaw < 3*pi/4:
        # print("loop_yaw: ", yaw)
        return yaw
    else:
        if yaw > 3*pi/4:
            yaw -= pi/2
        if yaw < pi/4:
            yaw += pi/2
        return yaw_converter(yaw)
    
def read_csv(filename):
    with open(filename, mode='r') as file:
        csv_reader = csv.reader(file)
        headers = next(csv_reader)
        data = []
        for row in csv_reader:
        # for row in csv_reader:
            data_row = [float(item) for item in row]
            data.append(data_row)
        return data
    
def minimum_rotation_radians(initial_angle, final_angles):
    # Normalize the initial angle to be within the range [-pi, pi)
    # print(f"initial_angle : {initial_angle}, final_angles: {final_angles}")
    initial_angle = (initial_angle + pi) % (2 * pi) - pi
    min_rotation = inf
    best_direction = ''
    # Iterate over each final angle
    for final_angle in final_angles:
        # Normalize the final angle
        final_angle = (final_angle + pi) % (2 * pi) - pi
        # Calculate clockwise and counterclockwise rotations
        clockwise = (final_angle - initial_angle) % (2 * pi)
        counterclockwise = (initial_angle - final_angle) % (2 * pi)
        # Find the minimum rotation for this pair
        if clockwise <= counterclockwise:
            rotation = clockwise
            direction = 'clockwise'
        else:
            rotation = counterclockwise
            direction = 'counterclockwise'
        # Update the smallest rotation found
        if rotation < min_rotation:
            min_rotation = rotation
            best_direction = direction
    return min_rotation, best_direction


class GenerationFunction():
    def __init__(self, message):
        self.message = message
        self.client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.client.wait_for_server()
        rospy.sleep(0.1)
        self.goal = TtsGoal()
    def speak(self):
        try:
            self.goal.rawtext.text = self.message
            self.goal.rawtext.lang_id = "en_GB"
            self.client.send_goal_and_wait(self.goal)
        except Exception as e:
            rospy.logerr(f"Failed to say Hi: {e}")
            return "Error trying to say Hi."


def publish_end_effector_pose():
    current_pose = get_current_end_effector_pose()
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "torso_lift_link"  # Adjust the frame_id as needed

    # Position
    pose_msg.pose.position.x = current_pose.p.x()
    pose_msg.pose.position.y = current_pose.p.y()
    pose_msg.pose.position.z = current_pose.p.z()

    # Orientation
    current_orientation = current_pose.M
    roll, pitch, yaw = current_orientation.GetRPY()
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose_msg.pose.orientation = Quaternion(*quaternion)

    ee_pose_pub.publish(pose_msg)

def publish_state_end_effector(message):
    number_msg = Float32MultiArray()
    # number_msg.data = [1.0, 2.0, 3.0, 4.0]
    number_msg.data = message
    State_pub.publish(number_msg)
    # rospy.loginfo("Published number array")


def move_arm(joint_angles, t):
        # Define the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        # Specify the joint names for arm and torso
        trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Define the joint target positions for arm and torso
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(t)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        arm_client.send_goal(goal)
        if arm_client.wait_for_result(rospy.Duration(t+1)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Arm completed successfully.")
        else:
            rospy.loginfo("Arm did not complete before the timeout.")


def move_arm_with_interp(joint_angles_list, duration):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    #traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names

    start_joint_list = [current_joint_positions[i] for i in range(number_of_joints)]
    end_joint_list = joint_angles_list

    num_interp = 100
    interp_lists = cosine_interp_list(start_joint_list, end_joint_list, num_interp)

    # add all joint lists
    for i in range(num_interp):

        this_joint_list = []
        for joint_index in range(len(interp_lists)):
            this_joint_list.append(interp_lists[joint_index][i])
    
        point = JointTrajectoryPoint()
        
        point.positions = this_joint_list
        point.time_from_start = rospy.Duration(i*(duration/(num_interp-1)))  # Adjust based on your requirements
        traj_msg.points.append(point)

    # print("message", traj_msg)
    
    # Publish the message
    arm_pub.publish(traj_msg)


def aruco_pose_callback(msg):
    global Arco_markers_updated_array

    # Process the message (just printing it for now)
    # print("Received ArUco pose:")
    # print("Marker ID:", msg.name[0])
    # print("Position (X, Y, Z):", msg.position[:3])
    Arco_markers_updated_array = np.array([msg.name[0], msg.position[0],msg.position[1]], dtype=object)
    print(Arco_markers_updated_array)
    # print(Arco_markers_updated_array)

    # Set the flag to True to stop receiving further messages

    # Optionally, you can stop the ROS node here
    # rospy.signal_shutdown("Received the ArUco pose once, shutting down.")



def run():
    global ik_solver_pos, desired_joint_positions, joint_names, number_of_joints, fk_solver, arm_pub, gripper_client, desired_frame, current_gripper_position, current_joint_positions, current_head_position, head_client, ee_pose_pub, State_pub, arm_client
    global data


    # Load the robot model from the parameter server
    robot_urdf = URDF.from_parameter_server()

    # Generate a KDL tree from the URDF model
    success, kdl_tree = treeFromUrdfModel(robot_urdf)
    if not success:
        rospy.logerr("Failed to extract KDL tree from URDF robot model.")
        exit(1)

    # Specify the chain: from base link to end-effector link
    base_link = "torso_lift_link"
    end_effector_link = "gripper_link"

    chain = kdl_tree.getChain(base_link, end_effector_link)

    # Initialize the IK solver
    ik_solver_pos = ChainIkSolverPos_LMA(chain)

    # Initialize the joint array with the number of joints
    number_of_joints = chain.getNrOfJoints()
    desired_joint_positions = JntArray(number_of_joints)
    current_joint_positions = JntArray(number_of_joints)

    # Initialize Forward Kinematics solver
    fk_solver = ChainFkSolverPos_recursive(chain)

    # List of joint names for the robot's arm - adjust this list to match your configuration
    joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
                   "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]

    current_gripper_position = [0, 0]
    current_head_position = [0, 0]


    # Publisher for controlling the robot's arm
    arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    # Action clients for the gripper and head
    gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("arm server connected.")

    # Subscribe to the current joint state
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.loginfo("Subscribed to joint states.")

    rospy.Subscriber('/aruco_pose_robot_camera', JointState, aruco_pose_callback)
    rospy.loginfo("Subscribed to aruco pose")

    # Wait to get first values to ensure everything is initialized properly
    rospy.wait_for_message('/joint_states', JointState)
    rospy.sleep(1.0)

    # Add a global publisher for end-effector pose
    ee_pose_pub = rospy.Publisher('/end_effector_pose', PoseStamped, queue_size=1)
    State_pub = rospy.Publisher('/ee_state_topic', Float32MultiArray, queue_size=1)


    System_input = int(sys.argv[1])
    Puzzle_number = int (sys.argv[2])
    Paricipant_ID = int (sys.argv[3])

    failure_number = int((System_input-1)/4)
    object_number = System_input % 4
    if object_number ==0:
        object_number = 4
    # print("object_number: ", object_number)
    # print("failure_number: ", failure_number)


    X, Y= data[object_number-1][1], data[object_number-1][2]
    # X, Y= 0.8, 0
    #X_3D = 0.8935*X+0.0055*Y+0.064
    # X_3D = 0.8943*X+0.00416*Y+0.0679-0.01
    X_3D = 0.8790*X+0.0162*Y + 0.073
    print(X, Y)
    if X>0.62:
        if Y>-0.18:
            X_3D = X_3D + 0.002  #Up Left
        else: 
            X_3D = X_3D + 0.003  #Up Right
    else:
        if Y<-0.18:
            X_3D = X_3D + 0.001  # Down Right
        else:
            X_3D = X_3D - 0.003 #Down Left


    Y_3D = 0.0455*X+0.9401*Y-0.0194
    X_2D = 0.888*X+0.0644
    Y_2D = 0.9319*Y+0.01
    X_Poly = 3.8927*(X) +0.3982*(Y) -2.2658*X**2 -0.4216*X*Y +0.2446*Y**2 - 0.9099
    yaw = data[object_number-1][6]
    X_pose = X_3D
    Y_pose = Y_3D-0.004

    # Destination Locations

    if Puzzle_number == 4: #Cat
        #Square
        final_square = [-3*pi/4, -pi/4, pi/4, 3*pi/4] #Orientation
        Destination_square = [0.753-0.009, 0.248+0.011+0.002]
        #Triangle 1
        final_triangle1 = [pi/2]
        Destination_triangle1 = [Destination_square[0]+0.0329, Destination_square[1]+0.0496+0.009]
        #Triangle 2
        final_triangle2 = [-pi/2]
        Destination_triangle2 = [Destination_square[0]-0.0327-0.011+0.002, Destination_square[1]+0.0496+0.007]
        #Trapezold
        final_trapezold = [-pi/4, 3*pi/4] #4
        Destination_trapezold = [Destination_square[0]-0.162, Destination_square[1]-0.1903-0.015-0.002]
        # Destination of Failure
        Destination_failure = [0.6, 0.2]
        Destination_failure = [Destination_square[0]-0.185, Destination_square[1]-0.1903]

    elif Puzzle_number == 1: #The rocket 
        #Square
        final_square = [-pi, -pi/2, 0, pi/2] #Orientation
        Destination_square = [0.686, 0.098]
        #Triangle 1
        final_triangle1 = [-3*pi/4]
        Destination_triangle1 = [Destination_square[0]-0.062-0.002, Destination_square[1]-0.015]
        #Triangle 2
        final_triangle2 = [pi/4]
        Destination_triangle2 = [Destination_square[0]-0.097+0.003, Destination_square[1]-0.0776+0.003]
        #Trapezold
        final_trapezold = [-pi/4, 3*pi/4] #4
        Destination_trapezold = [Destination_square[0]+0.092-0.007, Destination_square[1]-0.0422+0.007]

        # Destination of Failure
        Destination_failure = [Destination_square[0]+0.092-0.004, Destination_square[1]-0.0422]

    elif Puzzle_number == 2: #The rabbit
        #Square
        final_square = [-pi, -pi/2, 0, pi/2] #Orientation
        Destination_square = [0.753-0.136-0.008-0.003, 0.248-0.002]
        #Triangle 1
        final_triangle1 = [pi/2]
        Destination_triangle1 = [Destination_square[0]+0.021, Destination_square[1]-0.124]
        #Triangle 2
        final_triangle2 = [pi/2]
        Destination_triangle2 = [Destination_square[0]+0.070-0.002, Destination_square[1]-0.221+0.002]
        #Trapezold
        final_trapezold = [-3*pi/4, pi/4] #4
        Destination_trapezold = [Destination_square[0]+0.068, Destination_square[1]+0.0848+0.006]
        # Destination of Failure
        # Destination_failure = [0.53, 0.12]
        Destination_failure = [Destination_square[0]+0.07, Destination_square[1]+0.0848+0.006]

    elif Puzzle_number == 5: #The dog
        #Square
        final_square = [-pi, -pi/2, 0, pi/2] #Orientation
        Destination_square = [0.753-0.0488, 0.248+0.0706]
        #Triangle 1
        final_triangle1 = [pi]
        Destination_triangle1 = [Destination_square[0]+0.079, Destination_square[1]-0.0454]
        #Triangle 2
        final_triangle2 = [pi/2]
        Destination_triangle2 = [Destination_square[0]-0.1075, Destination_square[1]-0.3209]
        #Trapezold
        final_trapezold = [-pi/4, 3*pi/4] #4
        Destination_trapezold = [Destination_square[0]-0.126, Destination_square[1]-0.056]
        # Destination of Failure
        Destination_failure = [0.6, 0.2]

    elif Puzzle_number == 3: #The turtle
        #Square
        final_square = [-pi, -pi/2, 0, pi/2] #Orientation
        Destination_square = [0.753-0.026+0.002, 0.248+0.075+0.002]
        #Triangle 1
        final_triangle1 = [-pi/2]
        Destination_triangle1 = [Destination_square[0]-0.158, Destination_square[1]-0.13-0.004]
        #Triangle 2
        final_triangle2 = [-pi/2]
        Destination_triangle2 = [Destination_square[0]-0.158, Destination_square[1]-0.28]
        #Trapezold
        final_trapezold = [-3*pi/4, pi/4] #4
        Destination_trapezold = [Destination_square[0]-0.038, Destination_square[1]-0.085]
        # Destination of Failure
        #Destination_failure = [0.6, 0.31]
        Destination_failure = [0.753-0.026+0.002, 0.248+0.094]

    elif Puzzle_number == 6: #Swan
        #Square
        final_square = [-pi, -pi/2, 0, pi/2] #Orientation
        Destination_square = [0.753-0.026+0.002, 0.248+0.075+0.002]
        #Triangle 1
        final_triangle1 = [-pi/2]
        Destination_triangle1 = [Destination_square[0]-0.158, Destination_square[1]-0.13-0.004]
        #Triangle 2
        final_triangle2 = [-pi/2]
        Destination_triangle2 = [Destination_square[0]-0.158, Destination_square[1]-0.28]
        #Trapezold
        final_trapezold = [-3*pi/4, pi/4] #4
        Destination_trapezold = [Destination_square[0]-0.038, Destination_square[1]-0.085]
        # Destination of Failure
        #Destination_failure = [0.6, 0.31]
        Destination_failure = [0.753-0.026+0.002, 0.248+0.094]
    

    if object_number == 1:
        rotation, direction = minimum_rotation_radians(yaw, final_square)
        Destination = Destination_square
    elif object_number == 4:
        rotation, direction = minimum_rotation_radians(yaw, final_trapezold)
        Destination = Destination_trapezold
    elif object_number == 2 or object_number == 3:
        rotation1, direction1 = minimum_rotation_radians(yaw, final_triangle1)
        rotation2, direction2 = minimum_rotation_radians(yaw, final_triangle2)
        if rotation1 < rotation2:
            rotation = rotation1
            direction = direction1
            Destination = Destination_triangle1
        elif rotation1 == rotation2:
            if object_number == 2:
                rotation = rotation1
                direction = direction1
                Destination = Destination_triangle1
            elif object_number == 3:
                rotation = rotation2
                direction = direction2
                Destination = Destination_triangle2
        else:
            rotation = rotation2
            direction = direction2
            Destination = Destination_triangle2


    # print(f'Minimum rotation is {rotation:.2f} radians {direction}.')

    if rotation > pi/2:
        print("Wrong Orientation")
    #     # if direction == 'counterclockwise':
    #     #     yaw_goal = yaw_converter(yaw) + pi/2
    #     # elif direction == 'clockwise':
    #     #     yaw_goal = yaw_converter(yaw) - pi/2
    yaw_goal = yaw_converter(yaw)
    # print("Yaw_goal: ", yaw_goal)


    if direction == 'counterclockwise':
        yaw_des = yaw_goal - rotation
    elif direction == 'clockwise':
        yaw_des = yaw_goal + rotation

    # print("Yaw: ", yaw_goal)

    # print(yaw_des)

    # update_head_position(-15, -15, 2)

    # # Specify the goal position and orientation for the end-effector
    # goal_position = [0.924*X+0.0298*Y+0.0491, -0.0453*X+1.0084*Y+0.0714, -0.2]  # Adjust as needed
    # goal_position = [X, Y, -0.2]  # Adjust as needed
    # goal_orientation = [0, 0, yaw_des]  # Adjust as needed
    # move_to_goal_position(goal_position, goal_orientation)



    update_head_position(-20, -40, 2)

    message = "It is my turn"
    gen_func = GenerationFunction(message)
    gen_func.speak()
    goal_orientation = [0, 0, yaw_goal]  # Adjust as needed
    goal_position = [X_pose, Y_pose, -0.2]  # Adjust as needed
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 0])
    move_to_goal_position(goal_position, goal_orientation, 4)
    rospy.sleep(4)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 1])
    update_gripper_position(0.09)
    rospy.sleep(0.7)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 2])
    # goal_position = [X_pose, Y_pose, -0.2]  # Adjust as needed
    # move_to_goal_position(goal_position, goal_orientation, 3)
    # rospy.sleep(3)
    print(Arco_markers_updated_array)
    X_aruco = Arco_markers_updated_array[2]
    Y_aruco = Arco_markers_updated_array[1]

    X_update = (203-X_aruco)*0.013/82
    Y_update = (343-Y_aruco)*0.01/65
    print(Y_update)

    goal_position = [X_pose+X_update, Y_pose+Y_update, -0.248]  # Adjust as needed
    move_to_goal_position(goal_position, goal_orientation, 3)
    rospy.sleep(3)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 3])
    update_gripper_position(0.03)

    rospy.sleep(0.7)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 4])
    if failure_number==1:
        rospy.sleep(15)
        publish_end_effector_pose()
        publish_state_end_effector([Puzzle_number, object_number, failure_number, 4.1])
        if Paricipant_ID < 13:
            message = "Sorry for the delay"
            gen_func = GenerationFunction(message)
            gen_func.speak()
    goal_position = [X_pose, Y_pose, -0.18]  # Adjust as needed
    move_to_goal_position(goal_position, goal_orientation, 3)
    rospy.sleep(3)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 5])
    if failure_number == 2:
        goal_position = [Destination_failure[0], Destination_failure[1], -0.18]  # Adjust as needed
        goal_orientation = [0, 0, yaw_des]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 5)
        update_head_position(10, -40, 4)
        rospy.sleep(1.5)
        publish_end_effector_pose()
        publish_state_end_effector([Puzzle_number, object_number, failure_number, 5.1])
        goal_position = [Destination_failure[0], Destination_failure[1], -0.247]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(4)
        publish_end_effector_pose()
        publish_state_end_effector([Puzzle_number, object_number, failure_number, 5.2])
        rospy.sleep(4)
        if Paricipant_ID < 13:
            message = "Sorry I made a mistake"
            gen_func = GenerationFunction(message)
            gen_func.speak()
        goal_position = [Destination_failure[0], Destination_failure[1], -0.18]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(3)
        publish_end_effector_pose()
        publish_state_end_effector([Puzzle_number, object_number, failure_number, 5.3])
    goal_position = [Destination[0], Destination[1], -0.18]  # Adjust as needed
    goal_orientation = [0, 0, yaw_des]  # Adjust as needed
    move_to_goal_position(goal_position, goal_orientation, 5)
    update_head_position(10, -40, 4)
    rospy.sleep(1.5)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 6])
    goal_position = [Destination[0], Destination[1], -0.2472]  # Adjust as needed
    move_to_goal_position(goal_position, goal_orientation, 3)
    rospy.sleep(4)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 7])
    update_gripper_position(0.05)
    rospy.sleep(0.7)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 8])
    goal_position = [Destination[0], Destination[1], -0.18]  # Adjust as needed
    move_to_goal_position(goal_position, goal_orientation, 3)
    rospy.sleep(3)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 9])

    rospy.wait_for_message("joint_states", JointState)
    rospy.sleep(0.3)
    up_joint_angles = [0.07, 0.47, -1.53, 1.74, 0.37, -1.37, 0.28]
    move_arm_with_interp(up_joint_angles, 5)
    update_head_position(0, -10, 1)  # Note: Assuming negative tilt means downwards
    if Puzzle_number !=3 and object_number !=4:
        message = "Now, it is your turn"
    elif Puzzle_number !=3 and object_number ==4:
        message = "Now, Next Puzzle"
    elif Puzzle_number ==3 and object_number ==1:
        message = "Now, Next Puzzle"
    elif Puzzle_number ==3 and object_number !=1:
        message = "Now, it is your turn"
    if Puzzle_number ==4 and object_number ==4:
        message = "Puzzles are finished!"
    gen_func = GenerationFunction(message)
    gen_func.speak()
    rospy.sleep(1)
    publish_end_effector_pose()
    publish_state_end_effector([Puzzle_number, object_number, failure_number, 10])




    # Get current end-effector position and orientation
    current_end_effector_pose = get_current_end_effector_pose()
    current_position = current_end_effector_pose.p
    current_orientation = current_end_effector_pose.M

    
    print(f"x: {current_position.x()}, y: {current_position.y()}, z: {current_position.z()}")

    # Calculate the new orientation by converting to RPY, adding the deltas, and converting back
    # roll, pitch, yaw = current_orientation.GetRPY()

    # print(f"Orientation - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

    save_option = 0
    if save_option == 1:
        # Define the path to your file
        file_path = '/home/pal/tiago_ws/src/woa_tiago/project/Data.txt'

        # Values you want to replace the first two zeros with
        value1 = current_position.x()  # Change this to your desired value
        value2 = current_position.y()  # Change this to your desired value

        # Open the file in append mode and add a new line with "0 0 0 0"
        with open(file_path, 'a') as file:
            file.write("\n0 0")  # Append new line at the end

        # Now, read all lines, modify the last one, and write everything back
        with open(file_path, 'r') as file:
            lines = file.readlines()  # Read all lines into a list

        # Modify the last line
        last_line = lines[-1].strip().split(' ')  # Split the last line into a list of its values
        last_line[0] = str(value1)  # Replace the first zero with value1
        last_line[1] = str(value2)  # Replace the second zero with value2
        lines[-1] = ' '.join(last_line)  # Join back into a string and assign back to the last line

        # Write everything back to the file
        with open(file_path, 'w') as file:
            file.writelines(lines)  # Write all lines back into the file



if __name__ == "__main__":
    global data
    # Initialize ROS node
    rospy.init_node('tiago_arm_teleop_position')
    file_name = "aruco_data.csv"
    data = read_csv(file_name)
    if len(data)==4:
        run()
    else:
        print("Aruco file is not correct")




