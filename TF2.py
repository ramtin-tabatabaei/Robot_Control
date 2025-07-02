#!/usr/bin/env python
import rospy
import tf
import tf_conversions
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from math import pi
from csv import writer
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
class CoordinateTranslator:
    def __init__(self):
        rospy.init_node('coordinate_translator', anonymous=True)
        # Initialize the action client for controlling the head's movement
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_client.wait_for_server()
        self.listener = tf.TransformListener()
        self.aruco_tf_pub = rospy.Publisher("/aruco_pose_tf", JointState, queue_size=1)
        self.aruco_sub = rospy.Subscriber("/aruco_pose", JointState, self.aruco_callback)
        self.aruco_pub = rospy.Publisher("/aruco_pose_TF", JointState, queue_size=1)
        rospy.sleep(0)  # Wait for the listener to get ready
        self.aruco_pose = np.zeros((4,7))
        self.array_counter = np.zeros(4)
        self.message_count = 0
        self.counter = 0
        # Set u CSV file
        self.csv_file = open('aruco_data.csv', "w", newline ='')
        self.csv_writer = writer(self.csv_file)
        self.csv_writer.writerow(['Id','x', 'y', 'z' , 'roll', 'pitch', 'yaw'])
        rospy.sleep(0.3)
    def publish_static_transformation(self, x, y, z, roll, pitch, yaw):
        # Create a StaticTransformBroadcaster object
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        # Create a TransformStamped message
        static_transform_stamped = TransformStamped()
        # Fill the message with the necessary data
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = '/xtion_rgb_optical_frame'
        static_transform_stamped.child_frame_id = '/aruco_marker_frame'
        static_transform_stamped.transform.translation.x = x
        static_transform_stamped.transform.translation.y = y
        static_transform_stamped.transform.translation.z = z
        quaternion = tf_conversions.transformations.quaternion_from_euler(roll/180*pi, pitch/180*pi, yaw/180*pi)
        static_transform_stamped.transform.rotation.x = quaternion[0]
        static_transform_stamped.transform.rotation.y = quaternion[1]
        static_transform_stamped.transform.rotation.z = quaternion[2]
        static_transform_stamped.transform.rotation.w = quaternion[3]
        # Send the transformation
        static_broadcaster.sendTransform(static_transform_stamped)
        rospy.sleep(0.01)
    def aruco_callback(self, msg: JointState):
        self.marker_id = msg.name
        pose_vec = msg.position
        self.publish_static_transformation(pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5])
        trans, rot, rot_euler = self.get_transformation()
        if int(self.marker_id[0]) < 5:
            self.message_count += 1
            # print(self.marker_id[0])
            # print(f"Message number{self.message_count}")
            if abs(float(rot_euler[0])) < 0.1:
                self.aruco_pose[int(self.marker_id[0])-1, 0] = self.marker_id[0]
                self.aruco_pose[int(self.marker_id[0])-1, 1] += trans[0]
                self.aruco_pose[int(self.marker_id[0])-1, 2] += trans[1]
                self.aruco_pose[int(self.marker_id[0])-1, 3] += trans[2]
                self.aruco_pose[int(self.marker_id[0])-1, 4] = rot_euler[0]
                self.aruco_pose[int(self.marker_id[0])-1, 5] = rot_euler[1]
                self.aruco_pose[int(self.marker_id[0])-1, 6] = rot_euler[2]
                self.array_counter[int(self.marker_id[0])-1] += 1
        if self.message_count >=200:
            # print(self.aruco_pose)
            # print(self.array_counter)
            for i in range(4):
                for j in range(1,4):
                    self.aruco_pose[i,j] = self.aruco_pose[i,j]/self.array_counter[i]
            # print(self.aruco_pose)
            if all(x > 0 for x in self.array_counter):
                print("1")
            else:
                print("TF Function failed")
            for i in range(4):
                self.csv_writer.writerow(self.aruco_pose[i])
                pose_msg = JointState()
                pose_msg.name = [str(self.aruco_pose[i, 0])]
                pose_msg.position = self.aruco_pose[i, 1:]
                self.aruco_pub.publish(pose_msg)
                rospy.sleep(0.1)
            self.csv_file.close()
            rospy.signal_shutdown("Done")
            return
    def get_transformation(self):
        (trans, rot) = self.listener.lookupTransform("/torso_lift_link", "/aruco_marker_frame", rospy.Time(0))
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        aruco_tf_msg = JointState()
        aruco_tf_msg.name = self.marker_id
        aruco_tf_msg.position = [trans[0], trans[1], trans[2], rot_euler[0], rot_euler[1], rot_euler[2]]
        self.aruco_tf_pub.publish(aruco_tf_msg)
        # print("Publish aruco_tf")
        return trans, rot, rot_euler
    def rotate_and_tilt_head(self, pan_degrees, tilt_degrees):
        # global head_client
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
        point.time_from_start = rospy.Duration(5)  # Adjust the duration as needed for your setup
        trajectory.points.append(point)
        goal.trajectory = trajectory
        self.head_client.send_goal(goal)
        self.head_client.wait_for_result()
    def run(self):
        rospy.spin()
def main():
    translator = CoordinateTranslator()
    translator.run()
if __name__ == '__main__':
    main()