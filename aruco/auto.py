#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def read_waypoints_from_file(file_path):
    waypoints = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            data = line.strip().split()
            waypoint = PoseStamped()
            waypoint.pose.position.x = float(data[0])
            waypoint.pose.position.y = float(data[1])
            waypoint.pose.position.z = float(data[2])
            waypoint.pose.orientation.x = float(data[3])
            waypoint.pose.orientation.y = float(data[4])
            waypoint.pose.orientation.z = float(data[5])
            waypoint.pose.orientation.w = float(data[6])
            waypoints.append(waypoint)
    return waypoints

def send_waypoints(waypoints):
    goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.sleep(2)  # Ensure publisher has time to establish connection

    for waypoint in waypoints:
        goal_pub.publish(waypoint)
        rospy.loginfo("Sent waypoint to move_base!")
        rospy.sleep(10)  # Wait for the robot to reach the waypoint

def main():
    rospy.init_node('waypoint_navigation')

    waypoints_file_path = rospy.get_param("~waypoints_file_path", "waypoints.txt")
    waypoints = read_waypoints_from_file(waypoints_file_path)
    send_waypoints(waypoints)

if __name__ == '__main__':
    main()
