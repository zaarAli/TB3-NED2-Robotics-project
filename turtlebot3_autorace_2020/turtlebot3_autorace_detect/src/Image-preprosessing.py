#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()
pub = rospy.Publisher('eql_img', Image, queue_size=10)


# def image_callback(msg):
#     print("Received an image!")
#     try:
#         cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
#     except CvBridgeError:
#         print('error!')
#     else:
#         img_eq = preprocess_image(cv2_img)
#         ros_image = bridge.cv2_to_imgmsg(img_eq, encoding="bgr8") 
#         pub.publish(ros_image)

# def adaptive_color_threshold(image, low_color, high_color):
#     # Convert the image to HSV color space
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
#     # Define a mask using inRange function
#     mask = cv2.inRange(hsv, low_color, high_color)

#     # Apply morphological operations to remove noise and smoothen the mask
#     kernel = np.ones((5,5), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

#     return mask

# def color_correct(image, alpha=1.0, beta=0.0):
#     # Apply alpha and beta to perform color correction
#     corrected_image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

#     return corrected_image

# def preprocess_image(image):
#     # Adjust brightness and contrast for color correction
#     corrected_image = color_correct(image, alpha=1.5, beta=30)

#     # Convert the image to grayscale for better luminance information
#     gray_image = cv2.cvtColor(corrected_image, cv2.COLOR_BGR2GRAY)

#     # Compute adaptive threshold for white regions
#     _, white_mask = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

#     # Define the color thresholds for yellow lanes
#     low_yellow = np.array([15, 100, 100], dtype=np.uint8)
#     high_yellow = np.array([30, 255, 255], dtype=np.uint8)

#     # Apply adaptive color thresholding for yellow lanes
#     yellow_mask = adaptive_color_threshold(corrected_image, low_yellow, high_yellow)

#     # Combine the masks to get the final binary image
#     combined_mask = cv2.bitwise_or(white_mask, yellow_mask)

#     # Apply the combined mask to the original image
#     result = cv2.bitwise_and(image, image, mask=combined_mask)

#     return result


# # If the above does not work, try using the code bellow:

def preprocess_image(img):
    hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    channels=cv2.split(hsv)
    # print (len(channels))
    cv2.equalizeHist(channels[2])
    cv2.merge(channels,hsv)
    cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR,img)
    return img

# # If this doesn't work, use this code:


# def preprocess_image(image):
#     # Convert the image to LAB color space
#     lab_image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

#     # Apply histogram equalization to the L channel (luminance)
#     lab_image[:,:,0] = cv2.equalizeHist(lab_image[:,:,0])

#     # Convert the image back to BGR color space
#     enhanced_image = cv2.cvtColor(lab_image, cv2.COLOR_LAB2BGR)

#     return enhanced_image


def main():
    rospy.init_node('image_listener')
    # rospy.Subscriber("/camera/image", Image, image_callback)
    rospy.Subscriber("/camera/image_projected_compensated", Image, image_callback)
    rospy.spin()
if __name__ == '__main__':
    main()