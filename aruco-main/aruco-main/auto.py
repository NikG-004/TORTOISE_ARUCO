#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)

        # Define the waypoints
        self.waypoints = [
            (0.006415150206678516, -0.004372240518355188),
            (1.2246919245142107, 0.01869983587781174),
            (3.4226676227284143, -0.04167749190940052),
            (1.5609731136867369, -0.2301644364059942),
            (1.3266906682396484, -0.1605974379916684),
            (-1.9180066605719794, 0.10003458089672336),
            (-2.170277208851199, 0.09644348441675338)
        ]

        self.waypoint_tolerance = 0.5
        self.current_waypoint_index = 0
        self.current_position = None

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def go_to_waypoint(self, waypoint):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.current_position:
            dx = waypoint[0] - self.current_position.x
            dy = waypoint[1] - self.current_position.y
            distance = np.sqrt(dx**2 + dy**2)

            if distance < self.waypoint_tolerance:
                return True

            twist = Twist()
            twist.linear.x = dx * 0.8
            twist.linear.y = dy * 0.8

            self.velocity_publisher.publish(twist)
            rate.sleep()

        return False

    def navigate(self):
        while not rospy.is_shutdown() and self.current_waypoint_index < len(self.waypoints):
            rospy.loginfo("Navigating to waypoint: x=%f, y=%f", 
                          self.waypoints[self.current_waypoint_index][0], 
                          self.waypoints[self.current_waypoint_index][1])
                          
            if self.go_to_waypoint(self.waypoints[self.current_waypoint_index]):
                self.current_waypoint_index += 1

        # Stop the robot
        twist = Twist()
        self.velocity_publisher.publish(twist)
        rospy.loginfo("Navigation complete.")

if __name__ == '__main__':
    navigator = WaypointNavigator()
    navigator.navigate()
