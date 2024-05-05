#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

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

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Detect ArUco markers in the CV2 image
            corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                # Draw the detected markers on the frame
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                # Print the decoded IDs to ROS logs
                for i in ids:
                    rospy.loginfo("Detected ArUco marker with ID: %s", i[0])

                # Display the image
                cv2.imshow(self.node_name, cv_image)
                cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    detector = ArucoDetector()
    rospy.spin()
