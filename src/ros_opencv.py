#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Initialize global variables and ROS bridge
bridge = CvBridge()
src = None

# Define color ranges for traffic light detection in HSV
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])
lower_green = np.array([35, 100, 100])
upper_green = np.array([85, 255, 255])

# Morphology kernel and area threshold for traffic light detection
morphology_kernel = np.ones((11, 11), np.uint8)
area_threshold = 4000

# GStreamer pipeline for video capture
gstreamer_pipeline = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
cap = cv2.VideoCapture(gstreamer_pipeline)

# Callback function for video stream
def video_callback(data):
    global src
    try:
        src = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("cvBridgeError: %s", e)

# Function to publish processed image
def publish_processed_image(image):
    try:
        ros_image = bridge.cv2_to_imgmsg(image, "bgr8")
        image_pub.publish(ros_image)
    except CvBridgeError as e:
        rospy.logerr("cvBridgeError while publishing image: %s", e)

# Traffic light detection algorithm
def detect_traffic_lights(image):
    blurred_src = cv2.GaussianBlur(image, (11, 11), sigmaX=0, sigmaY=0)
    hsv_src = cv2.cvtColor(blurred_src, cv2.COLOR_BGR2HSV)

    red_mask1 = cv2.inRange(hsv_src, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv_src, lower_red2, upper_red2)
    red_mask = red_mask1 | red_mask2

    green_mask = cv2.inRange(hsv_src, lower_green, upper_green)
    total_mask = red_mask | green_mask
    dilated_mask = cv2.dilate(total_mask, morphology_kernel, iterations=1)

    red_area_total = 0
    green_area_total = 0

    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in red_contours:
        if cv2.contourArea(contour) < 100:
            continue
        area = cv2.contourArea(contour)
        red_area_total += area
        
        # Draw rectangle around red contour
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)  # Red rectangle

    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in green_contours:
        if cv2.contourArea(contour) < 100:
            continue
        area = cv2.contourArea(contour)
        green_area_total += area
        
        # Draw rectangle around green contour
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Green rectangle

    if red_area_total > area_threshold:
        rospy.loginfo("Red area detected, publishing 'red'")
        pub.publish("red")
    elif green_area_total > area_threshold:
        rospy.loginfo("Green area detected, publishing 'green'")
        pub.publish("green")
    else:
        rospy.loginfo("No significant area detected, publishing 'no'")
        pub.publish("no")

# Crosswalk detection algorithm
def detect_crosswalk(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    morphed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(morphed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    detected_tiles = []
    all_points = []  # To collect all bounding box points

    min_area = 500
    min_width = 50

    for c in contours:
        area = cv2.contourArea(c)
        if area > min_area:
            epsilon = 0.02 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, epsilon, True)

            if len(approx) >= 4:
                if is_trapezoid([pt[0] for pt in approx]):
                    x, y, w, h = cv2.boundingRect(c)
                    if w > min_width:
                        detected_tiles.append(approx)
                        all_points.extend([pt[0] for pt in approx])  # Collect points for convex hull
                        cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)  # Draw green tile

    if len(detected_tiles) >= 3:
        if all_points:
            all_points = np.array(all_points)
            hull = cv2.convexHull(all_points)
            
            # Filter out distant points
            dist_thresh = 200
            filtered_points = [point for point in all_points if np.linalg.norm(point - np.mean(all_points, axis=0)) < dist_thresh]
            if filtered_points:
                filtered_points = np.array(filtered_points)
                hull = cv2.convexHull(filtered_points)
                cv2.drawContours(image, [hull], 0, (0, 0, 255), 2)  # Draw red convex hull
        
        cv2.putText(image, 'Crosswalk Detected', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
	rospy.loginfo("crosswalk detected!")
        cross_pub.publish("crosswalk")  # 횡단보도가 인식된 경우 "crosswalk" 발행
    else:
        cv2.putText(image, 'Not Enough Tiles', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
	rospy.loginfo("no crosswalk!")
        cross_pub.publish("no")  # 횡단보도가 인식되지 않은 경우 "no" 발행


# Function to check if the given vertices form a trapezoid
def is_trapezoid(vertices):
    if len(vertices) != 4:
        return False

    vertices = sorted(vertices, key=lambda x: x[1])

    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = [tuple(pt) for pt in vertices]

    def slope(p1, p2):
        return (p2[1] - p1[1]) / (p2[0] - p1[0]) if (p2[0] - p1[0]) != 0 else float('inf')

    top_slope = slope((x1, y1), (x2, y2))
    bottom_slope = slope((x3, y3), (x4, y4))

    return abs(top_slope - bottom_slope) < 0.1

# ROS node initialization
rospy.init_node('ros_vision_opencv')
rospy.Subscriber('/opencv_topic', Image, video_callback)
pub = rospy.Publisher('/color_detected', String, queue_size=10)
cross_pub = rospy.Publisher('/cross_detected', String, queue_size=10)
image_pub = rospy.Publisher('/processed_topic', Image, queue_size=10)

# Main loop
rate = rospy.Rate(10)  # 10 Hz

while not rospy.is_shutdown():
    if src is None:
        continue  # Skip processing if src is None

    # Process for traffic light detection
    detect_traffic_lights(src)

    # Process for crosswalk detection
    detect_crosswalk(src)

    # Publish the processed image (optional)
    publish_processed_image(src)

    rate.sleep()

cv2.destroyAllWindows()

