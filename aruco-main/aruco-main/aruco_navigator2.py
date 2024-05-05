#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class ArucoDetector:
    def __init__(self):
        self.node_name = "aruco_detector"
        rospy.init_node(self.node_name)

        # Bridge to convert ROS Image type to OpenCV Image
        self.bridge = CvBridge()

        # Initialize the ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ROS Subscriber for the robot's camera feed
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        
        # ROS Subscriber for robot's pose
        self.pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, self.pose_callback)

        # Robot's current pose
        self.robot_pose = None
        self.detected_ids = set()

        # List to store waypoints (marker_id, pose) tuples
        self.waypoints = []

    def pose_callback(self, msg):
        self.robot_pose = msg  # Store the entire PoseStamped message

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                for i in ids:
                    if i[0] not in self.detected_ids:
                        self.detected_ids.add(i[0])  
                        rospy.loginfo("Detected ArUco marker with ID: %s", i[0])
                        
                        # Store the robot's current pose along with the marker ID when an ArUco marker is detected
                        if self.robot_pose:
                            self.waypoints.append((i[0], self.robot_pose))

                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.imshow(self.node_name, cv_image)
                cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(e)

    def save_waypoints_to_file(self):
        with open("waypoints9.txt", "w") as file:
            for marker_id, waypoint in self.waypoints:
                pos = waypoint.pose.position
                ori = waypoint.pose.orientation
                file.write(f"Marker ID: {marker_id}, Position: {pos.x},{pos.y},{pos.z}, Orientation: {ori.x},{ori.y},{ori.z},{ori.w}\n")

if __name__ == '__main__':
    detector = ArucoDetector()
    rospy.on_shutdown(detector.save_waypoints_to_file)
    rospy.spin()
