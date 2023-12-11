#!/usr/bin/env python3

"""
ArucoDetection Node

This node subscribes to a compressed image topic from a camera, detects ArUco markers, estimates their pose,
computes the distance to each marker, and publishes the marker images along with their distances and IDs.

"""

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64, Int32, Bool  # Import Bool message type
import cv2
from cv2 import aruco
import numpy as np

class ArucoDetection:
    def __init__(self):
        """
        Initialize ArucoDetection node.

        This method initializes the ROS node, sets up subscribers and publishers, and defines camera calibration parameters.
        """
        rospy.init_node('aruco_detection', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber for compressed image topic
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback)

        # Publishers for ArUco marker images, distances, and IDs
        self.marker_pub = rospy.Publisher('/aruco_markers', Image, queue_size=10)
        self.distance_pub = rospy.Publisher('/aruco_distance', Float64, queue_size=10)
        self.id_pub = rospy.Publisher('/aruco_id', Int32, queue_size=10)

        # Subscriber for robot status
        self.robot_status_sub = rospy.Subscriber('/robot_status', Bool, self.robot_status_callback)

        # Camera calibration parameters
        self.camera_matrix = np.array([[153.53337, 0.0, 150.50761],
                                       [0.0, 153.62598, 118.64754],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeff = np.array([-0.318094, 0.090092, 0.000346, -0.000410, 0.0])
        self.last_published_id = None
        self.stopped = False  # Flag to stop Aruco detection

    def image_callback(self, msg):
        try:
            if not self.stopped:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

                aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
                parameters = aruco.DetectorParameters_create()
                corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                if ids is not None:
                    # Take the first detected ArUco marker
                    aruco_id = ids[0][0]

                    if aruco_id != self.last_published_id:
                        # Publish the ID only if it has changed
                        print(f"Detected ArUco marker {aruco_id}")
                        self.id_pub.publish(Int32(aruco_id))
                        self.last_published_id = aruco_id

                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, self.camera_matrix, self.dist_coeff)

                    aruco.drawDetectedMarkers(cv_image, corners, ids)

                    x, y = int(corners[0][0][0][0]), int(corners[0][0][0][1])

                    cv2.putText(cv_image, f"ID: {aruco_id}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

                    distance = np.linalg.norm(tvecs[0])
                    print(f"Distance to ArUco marker {aruco_id}: {distance} meters")

                    self.distance_pub.publish(Float64(distance))

                aruco_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                self.marker_pub.publish(aruco_image_msg)
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

    def robot_status_callback(self, msg):
        if msg.data:  # If robot has stopped
            self.stopped = True
            rospy.loginfo("Aruco detection stopped for 50 seconds.")
            rospy.sleep(50)  # Stop Aruco detection for 50 seconds
            self.stopped = False
            rospy.loginfo("Aruco detection resumed.")

if __name__ == '__main__':
    try:
        # Create an instance of the ArucoDetection class and spin the node
        aruco_detection = ArucoDetection()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
