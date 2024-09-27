#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from jetson_inference import detectNet
import jetson_utils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

model_path = 'traffic_signs'
gstreamer_pipeline = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

cap = cv2.VideoCapture(gstreamer_pipeline)
if not cap.isOpened():
    rospy.logerr("CSI 카메라 열기 실패.")
    exit()

net = detectNet(argv=['--model=/home/farm/catkin_ws/src/ros_vision/src/models/' + model_path + '/ssd-mobilenet.onnx', 
                       '--labels=/home/farm/catkin_ws/src/ros_vision/src/models/' + model_path + '/labels.txt', 
                       '--input-blob=input_0', 
                       '--output-cvg=scores', 
                       '--output-bbox=boxes'])

rospy.init_node('ros_vision_detectnet')
data_pub = rospy.Publisher('inference', String, queue_size=1)
opencv_publisher = rospy.Publisher('/opencv_topic', Image, queue_size=10)
inference_publisher = rospy.Publisher('/inference_topic', Image, queue_size=10)
bridge = CvBridge()

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret or frame is None:
        rospy.logerr("비디오 캡처에서 프레임을 읽는 데 실패했습니다.")
        continue

    if not isinstance(frame, np.ndarray):
        rospy.logerr("프레임이 NumPy 배열이 아닙니다. 타입: %s", type(frame))
        continue

    try:
        opencv_topic = bridge.cv2_to_imgmsg(frame, "bgr8")
        opencv_publisher.publish(opencv_topic)
    except TypeError as e:
        rospy.logerr("cv2_to_imgmsg 실패: %s", str(e))
        continue

    img_cuda = jetson_utils.cudaFromNumpy(frame)
    detections = net.Detect(img_cuda, overlay="box,labels,conf")
    for detection in detections:
        confidence = detection.Confidence
        left = int(detection.Left)
        right = int(detection.Right)
        top = int(detection.Top)
        bottom = int(detection.Bottom)
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        label = net.GetClassDesc(detection.ClassID)
        text = "%s (%.1f%%)" % (label, confidence * 100)
        cv2.putText(frame, text, (left, top-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        data_list = [label, confidence * 100]
        data = ','.join(map(str, data_list))
	if confidence >= 0.8:
            data_pub.publish(data)
	    rospy.loginfo("Published: %s with confidence: %.1f%%" % (label, confidence * 100))

 
    inference_topic = bridge.cv2_to_imgmsg(frame, "bgr8")
    inference_publisher.publish(inference_topic)

    #cv2.imshow('Object Detection', frame)
    #if cv2.waitKey(1) == ord('q'):
    #    break

try:
    rosnode.kill_nodes(['ros_detectnet_subscriber'])
except:
    rospy.loginfo("노드 'ros_detectnet_subscriber'를 종료할 수 없습니다.")
try:
    rosnode.kill_nodes(['ros_detectnet'])
except:
    rospy.loginfo("노드 'ros_detectnet'를 종료할 수 없습니다.")
cv2.destroyAllWindows()

