#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)

        # Define the waypoints with IDs
        self.waypoints = [
            {"ID": 1, "Position": (0.00641, -0.004)},
            {"ID": 2, "Position": (1.22469, 0.0186)},
            {"ID": 5, "Position": (5.15810,1.69)},
            {"ID": 4, "Position": (-0.121,1.437)},
            
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
        dx = waypoint["Position"][0] - self.current_position.x
        dy = waypoint["Position"][1] - self.current_position.y
        distance_x = np.abs(dx)
        distance_y = np.abs(dy)

        twist = Twist()

        if distance_x > self.waypoint_tolerance:
            twist.linear.x = np.sign(dx) * 0.8  # Move in x direction
            twist.linear.y = 0.0
        elif distance_y > self.waypoint_tolerance:
            twist.linear.x = 0.0
            twist.linear.y = np.sign(dy) * 0.8  # Move in y direction
        else:
            # Both x and y are within tolerance, waypoint reached
            return True

        self.velocity_publisher.publish(twist)
        rate.sleep()

     return False

    def navigate(self):
        while not rospy.is_shutdown() and self.current_waypoint_index < len(self.waypoints):
            current_waypoint = self.waypoints[self.current_waypoint_index]
            rospy.loginfo(f"Navigating to Marker ID: {current_waypoint['ID']} with Position: x={current_waypoint['Position'][0]}, y={current_waypoint['Position'][1]}")
                          
            if self.go_to_waypoint(current_waypoint):
                self.current_waypoint_index += 1

        # Stop the robot
        twist = Twist()
        self.velocity_publisher.publish(twist)
        rospy.loginfo("Navigation complete.")

if __name__ == '__main__':
    navigator = WaypointNavigator()
    navigator.navigate()