#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped  # Import PoseStamped message type

class ArucoDetector:
    def __init__(self):
        self.node_name = "aruco_detector"
        rospy.init_node(self.node_name)

        # Bridge to convert ROS Image type to OpenCV Image
        self.bridge = CvBridge()

        # Initialize the ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ROS Subscriber for the robot's camera feed and robot pose
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, self.pose_callback) 

        # Dictionary to store detected markers and robot pose
        self.detected_markers = {}

        # Robot's current pose
        self.robot_pose = None

        # List to store waypoints
        self.waypoints = []

    def pose_callback(self, msg):
        self.robot_pose = msg  # Store the entire PoseStamped message

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                for i, marker_id in enumerate(ids):
                    # Check if marker ID is already stored
                    if marker_id[0] not in self.detected_markers:
                        self.detected_markers[marker_id[0]] = {'corners': corners[i], 'pose': self.robot_pose}
                        rospy.loginfo("Stored ArUco marker ID: %s with corners: %s and robot pose: %s", marker_id[0], corners[i], self.robot_pose)
                        # Store the waypoint
                        self.waypoints.append(self.robot_pose)

                # Draw the detected markers on the frame
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.imshow(self.node_name, cv_image)
                cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(e)

    def save_waypoints_to_file(self, file_path):
        with open(file_path, "w") as f:
            for waypoint in self.waypoints:
                f.write(f"Position: {waypoint.pose.position}\n")
                f.write(f"Orientation: {waypoint.pose.orientation}\n")
                f.write("\n")

if __name__ == '__main__':
    detector = ArucoDetector()

    # Run the node until shutdown
    try:
        rospy.spin()
    except KeyboardInterrupt:
         # Save waypoints to a file before exiting
        detector.save_waypoints_to_file("waypoints.txt")