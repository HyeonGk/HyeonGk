#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from std_msgs.msg import String
import numpy as np

# class, accuracy
label = ""
accuracy = 0.0


def get_inference(data):
	global label, accuracy
	detect = data.data.split(',')
	label = detect[0]
	accuracy = float(detect[1])
	rospy.loginfo("class= %s || accuracy= %f" % (label, accuracy))

rospy.init_node('ros_detectnet_subscriber')

rospy.Subscriber('/inference', String, get_inference)

while not rospy.is_shutdown(): pass
