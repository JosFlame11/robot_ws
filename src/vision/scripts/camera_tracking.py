#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

# Initialize the CvBridge
bridge = CvBridge()

# Define lower and upper bounds for the color yellow in HSV space
lower_bound = np.array([20, 60, 50])
upper_bound = np.array([30, 255, 255])

# Width of the frame (since we set it to 640)
frame_width = 320
# Calculate sector width
sector_width = frame_width // 5

# Sector labels
sector_labels = [-2, -1, 0, 1, 2]

# Initialize the publisher
cmd_vel_pub = None

def callbackFunction(ros_image):
    global cmd_vel_pub

    try:
        # Convert the ROS image message to an OpenCV image
        frame = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Convert the captured frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(hsv, (59, 59), 0)

    # Create a mask using the specified color bounds
    mask = cv2.inRange(blurred, lower_bound, upper_bound)

    # Apply morphological operations to remove noise and fill gaps
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    sector_label = None
    # Draw contours and determine sector
    if len(contours) != 0:
        for contour in contours:
            # Only consider contours with an area greater than 500
            if cv2.contourArea(contour) > 500:
                cv2.drawContours(frame, [contour], -1, (255, 0, 0), 3)
                
                # Calculate the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Determine the sector based on the x-coordinate of the centroid
                    sector_index = cX // sector_width
                    if sector_index >= 5:
                        sector_index = 4
                    sector_label = sector_labels[sector_index]

                    # Display the sector number on the frame
                    cv2.putText(frame, str(sector_label), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    # Publish the sector position as a Twist message
                    twist_msg = Twist()
                    #twist_msg.linear.x = 0
                    twist_msg.angular.z = sector_label * 0.3  # Adjust the multiplier as needed
                    cmd_vel_pub.publish(twist_msg)
                    break

    # Display the original frame and the mask (for debugging)
    cv2.imshow('frame', frame)
    cv2.waitKey(3)  # A small delay to allow OpenCV to display the image

def main():
    global cmd_vel_pub

    # Initialize the ROS node
    rospy.init_node('image_processing_node', anonymous=True)

    # Initialize the publisher
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Initialize the subscriber
    rospy.Subscriber('video_topic', Image, callbackFunction)
    
    rospy.spin()

if __name__ == '__main__':
    main()
