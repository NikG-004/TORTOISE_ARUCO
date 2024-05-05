#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class LaserScanToPointCloud:

    def __init__(self):
        self.node_name = "laserscan_to_pointcloud"
        
        rospy.init_node(self.node_name)

        self.laser_proj = LaserProjection()
        self.point_cloud_pub = rospy.Publisher("/cloud", PointCloud2, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def scan_callback(self, scan_msg):
        # Convert laser scan to point cloud
        cloud_msg = self.laser_proj.projectLaser(scan_msg)
        self.point_cloud_pub.publish(cloud_msg)

if __name__ == '__main__':
    LaserScanToPointCloud()
    rospy.spin()
