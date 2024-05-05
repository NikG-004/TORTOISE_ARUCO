#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry

def odometry_callback(msg):
    br = tf.TransformBroadcaster()
    translation = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    rotation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    br.sendTransform(translation, rotation, rospy.Time.now(), "base_link", "odom")

rospy.init_node('odom_to_tf_broadcaster')
rospy.Subscriber('/odom', Odometry, odometry_callback)
rospy.spin()
