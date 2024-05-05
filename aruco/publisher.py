#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # Import the Odometry message

class RobotPosePublisher:
    def __init__(self):
        self.node_name = "robot_pose_publisher"
        rospy.init_node(self.node_name)

        # ROS Publisher for the robot's pose
        self.pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=10)

        # ROS Subscriber for the robot's odometry (modify the topic name)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Initialize a PoseStamped message
        self.robot_pose_msg = PoseStamped()

        # Set the rate at which to publish the pose (adjust as needed)
        self.publish_rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, odom_msg):
        # Obtain the current robot pose from the odometry message
        # Modify this part based on your specific odometry message structure
        self.robot_pose_msg.header.stamp = odom_msg.header.stamp
        self.robot_pose_msg.header.frame_id = "odom"  # Replace with the appropriate frame ID
        self.robot_pose_msg.pose = odom_msg.pose.pose

    def publish_robot_pose(self):
        while not rospy.is_shutdown():
            # Publish the robot's pose
            self.pose_pub.publish(self.robot_pose_msg)

            # Sleep to maintain the publish rate
            self.publish_rate.sleep()

if __name__ == '__main__':
    pose_publisher = RobotPosePublisher()
    try:
        pose_publisher.publish_robot_pose()
    except rospy.ROSInterruptException:
        pass
