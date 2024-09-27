#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import cv2
import subprocess
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QGridLayout
from PyQt5.QtCore import QTimer, QSize, QMetaObject, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import QtWidgets, QtCore
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

class VideoSubscriber(QWidget):
    def __init__(self):
        super(VideoSubscriber, self).__init__()
        self.initUI()
	
        # ROS 초기화

        self.pub_position = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.sub_result = rospy.Subscriber("/move_base/result",MoveBaseActionResult,self.callback_act,queue_size=1)
	self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.position = PoseStamped()
	self.saved_position = None

	self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

	# CvBridge 객체 생성
        self.bridge = CvBridge()

##########################################################################
#토픽 관련
##########################################################################
        self.class_subscriber = rospy.Subscriber("/inference", String, self.class_callback)
        self.image_inference = rospy.Subscriber("/inference_topic", Image, self.callback_inference)
        self.image_opencv = rospy.Subscriber("/processed_topic", Image, self.callback_opencv)
        self.crosswalk_subscriber = rospy.Subscriber("/cross_detected", String, self.crosswalk_callback)
        self.color_subscriber = rospy.Subscriber("/color_detected", String, self.color_callback)
	self.position_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_current_position)

        # 이미지 데이터를 저장할 변수 초기화
        self.frame_inference = None
        self.frame_opencv = None
       
        # waypoint count
        self.waypoint = 0
        self.saved_waypoint = None
        self.teleop_mode = False
        self.navigation_mode = False



        self.crosswalk_flag = False
        self.human_detected = False
	self.red_flag = False
	self.green_flag = False
        self.current_obstacle = "none"


        self.human_detected_count = 0
        self.stop_detected_count = 0
	self.crosswalk_count = 0
	self.duration = 5
	self.current_position = None

        # 타이머를 이용해 비디오 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateFrames)
        self.timer.start(30)

    def initUI(self):
        self.setWindowTitle("ㅅ 왜 안됨")
        self.setGeometry(100, 100, 800, 600)

        # 비디오 피드 레이블 생성
        self.video_inference = QLabel(self)
        self.video_opencv = QLabel(self)



##########################################################################
#버튼 관련
##########################################################################
        # 버튼 생성
        self.roomb_button = QPushButton('START', self)
        self.forward_button = QPushButton('Forward', self)
        self.backward_button = QPushButton('Backward', self)
        self.left_button = QPushButton('Left', self)
        self.right_button = QPushButton('Right', self)
        self.stop_button = QPushButton('Stop', self)
        self.exit_button = QPushButton('Exit', self)
	self.traffic_button = QPushButton('traffic_mode', self)

        # 버튼 크기 설정
        self.forward_button.setFixedSize(QSize(60, 60))
        self.backward_button.setFixedSize(QSize(60, 60))
        self.left_button.setFixedSize(QSize(60, 60))
        self.right_button.setFixedSize(QSize(60, 60))
        self.stop_button.setFixedSize(QSize(60, 60))
        self.roomb_button.setFixedSize(QSize(100, 100))
        self.exit_button.setFixedSize(QSize(80, 40))
	self.traffic_button.setFixedSize(QSize(80, 40))

        # 버튼 클릭 이벤트 핸들러 설정
        self.roomb_button.clicked.connect(self.roomb)
        self.forward_button.clicked.connect(lambda: self.activate_teleop_and_move('forward'))
        self.backward_button.clicked.connect(lambda: self.activate_teleop_and_move('backward'))
        self.left_button.clicked.connect(lambda: self.activate_teleop_and_move('left'))
        self.right_button.clicked.connect(lambda: self.activate_teleop_and_move('right'))
        self.stop_button.clicked.connect(self.deactivate_teleop_and_stop)
        self.exit_button.clicked.connect(self.exit_process)
	self.traffic_button.clicked.connect(self.change_traffic)

        # 색상 감지 결과를 표시할 QLabel
        self.status_label = QtWidgets.QLabel(self)
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.status_label.setText("색상 감지를 기다리고 있습니다...")

        # 클래스 감지 결과를 표시할 QLabel
        self.class_status_label = QtWidgets.QLabel(self)
        self.class_status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.class_status_label.setText("클래스 감지를 기다리고 있습니다...")

        # teleop 상태를 표시할 QLabel
        self.obstacle_status_label = QtWidgets.QLabel(self)
        self.obstacle_status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.obstacle_status_label.setText("현재 대기 중인 장애물 | 시작 : none")

          #  정지 상태를 표시할 QLabel
        self.stop_status_label = QtWidgets.QLabel(self)
        self.stop_status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.stop_status_label.setText("stop or resume")

          # 레이아웃 설정
        grid = QGridLayout()
        grid.addWidget(self.video_inference, 0, 2, 1, 2)  # 비디오 피드 위치
        grid.addWidget(self.video_opencv, 0, 4, 1, 2)  # 두 번째 비디오 위치
        grid.addWidget(self.forward_button, 2, 1)
        grid.addWidget(self.backward_button, 4, 1)
        grid.addWidget(self.left_button, 3, 0)
        grid.addWidget(self.right_button, 3, 2)
        grid.addWidget(self.stop_button, 3, 1)
        grid.addWidget(self.roomb_button, 2, 3)
	grid.addWidget(self.traffic_button, 2, 4)
        grid.addWidget(self.exit_button, 3, 3)  # Exit 버튼 추가
	

        # 상태 레이블을 새로운 행에 추가
        grid.addWidget(self.status_label, 1, 4, 1, 3)
	self.status_label.setStyleSheet("font-size: 21px;")
        grid.addWidget(self.obstacle_status_label, 2, 4, 1, 3)
	self.obstacle_status_label.setStyleSheet("font-size: 21px;")
        grid.addWidget(self.class_status_label, 3, 4, 1, 3)
	self.class_status_label.setStyleSheet("font-size: 21px;")
	grid.addWidget(self.stop_status_label, 4, 4, 1, 3)
	self.stop_status_label.setStyleSheet("font-size: 21px;")
        self.setLayout(grid)        
       
##########################################################################
#영상 관련
##########################################################################
    def callback_inference(self, data):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        self.frame_inference = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def callback_opencv(self, data):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        self.frame_opencv = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def updateFrames(self):
        if self.frame_inference is not None:
            self.displayImage(self.frame_inference, self.video_inference)

        if self.frame_opencv is not None:
            self.displayImage(self.frame_opencv, self.video_opencv)

    def displayImage(self, img, label):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = img.shape
        bytes_per_line = ch * w
        qimg = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        label.setPixmap(pixmap)


##########################################################################
#방향키 설정
##########################################################################
    def activate_teleop_and_move(self, direction):
        if not self.teleop_mode:
            self.teleop_mode = True
            self.update_teleop_status(Twist())  # 상태 업데이트

        twist = Twist()
        if direction == 'forward':
            twist.linear.x = 0.3
            twist.angular.z = 0.0
        elif direction == 'backward':
            twist.linear.x = -0.3
            twist.angular.z = 0.0
        elif direction == 'left':
            twist.linear.x = 0.0
            twist.angular.z = 0.2
        elif direction == 'right':
            twist.linear.x = 0.0
            twist.angular.z = -0.2

        self.cmd_vel_pub.publish(twist)
        self.update_teleop_status(twist)
        print("Teleop 명령 실행 중: {}".format(direction))

    def deactivate_teleop_and_stop(self):
        if self.teleop_mode:
            self.teleop_mode = False
            self.teleop_control('stop')
            self.update_teleop_status(Twist())  # 상태 업데이트

    def teleop_control(self, command):
        twist = Twist()
        if command == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print("Teleop 명령 실행 중: {}".format(command))

    def update_teleop_status(self, twist):
        linear_x = twist.linear.x
        angular_z = twist.angular.z


##########################################################################
#이동 커맨드 관련
##########################################################################
    def move_command(self, pos_x, pos_y, ori_z, ori_w):
        if self.teleop_mode:
            print("Teleop 모드가 활성화되어 있어 네비게이션을 수행할 수 없습니다.")
            return

        self.position.header.seq = 0
        self.position.header.stamp = rospy.Time.now()
        self.position.header.frame_id = 'map'

        self.position.pose.position.x = pos_x
        self.position.pose.position.y = pos_y
        self.position.pose.position.z = 0.0

        self.position.pose.orientation.x = 0
        self.position.pose.orientation.y = 0
        self.position.pose.orientation.z = ori_z
        self.position.pose.orientation.w = ori_w

        self.pub_position.publish(self.position)
        self.navigation_mode = True

    def choice_waypoint(self, num):
        if self.teleop_mode:
            print("Teleop 모드가 활성화되어 있어 네비게이션을 수행할 수 없습니다.")
            return
        if num == 11:
            self.move_command(-0.873784, 10.6438, -0.710349, 0.70385)
        elif num == 12:
            self.move_command(-1.2, -4.8811, -0.701997, 0.71218)
        elif num == 2:
            self.move_command(1.95, -6.0357, 0.0097016, 0.999995)
	elif num == 21:
            self.move_command(-1.2, -4.8811, -0.701997, 0.71218)
        elif num == 22:
            self.move_command(-0.920461, -12.0382, -0.69908, 0.715043)
        elif num == 3:
            self.move_command(1.75378, -14.7419, -0.0177228, 0.999843)
        else:
            print("잘못된 웨이포인트 번호입니다.")

##########################################################################
#이동 관련 callback
##########################################################################
    def callback_act(self, result):
        if result.status.status == 3:  # 목표 도달
            if self.navigation_mode:
                if self.waypoint == 11:
                    self.waypoint = 12
                    self.choice_waypoint(self.waypoint)
                    print("wp 12")
                elif self.waypoint == 12:
                    self.waypoint = 2
                    self.choice_waypoint(self.waypoint)
                    print("wp 2")
		elif self.waypoint == 2:
                    self.waypoint = 21
                    self.choice_waypoint(self.waypoint)
                    print("wp 21")
                elif self.waypoint == 21:
                    self.waypoint = 22
                    self.choice_waypoint(self.waypoint)
                    print("wp 22")
		elif self.waypoint == 22:
                    self.waypoint = 3
                    self.choice_waypoint(self.waypoint)
                    print("wp 3")
                else:
                    # 다른 웨이포인트 처리
                    pass
                if self.waypoint == 3:
                    self.navigation_mode = False
	 
##########################################################################
#ai 클래스 구분
##########################################################################
    def class_callback(self, msg):
        data = msg.data
        if "stop" in data:
            if self.current_obstacle == "crosswalk":
                self.class_status_label.setText("정지 신호 감지")
                self.stop_movement(5)  
                self.current_obstacle = "stop"
		self.update_obstacle_status()
        elif "human" in data:
            if self.current_obstacle == "none":
                self.human_detected_count += 1
                if self.human_detected_count >= 1:
                    self.human_detected = True
                    self.class_status_label.setText("사람 신호 감지")
                    print("detected human")
                    self.human_detected_count = 0  # Reset count after action
        else:
            if self.current_obstacle == "none":
                if self.human_detected:
                    self.human_detected = True #일단 사람은 항상 인식된다고 쳤음
                    print("사람 없음")

##########################################################################
#opencv 횡단보도 구분
##########################################################################
    def crosswalk_callback(self, msg):
        cross = msg.data
	if cross == "crosswalk":
	    self.crosswalk_count += 1
	    if self.crosswalk_count >= 1:
	        self.crosswalk_flag = True

        if self.crosswalk_flag == True and self.current_obstacle == "none":
	    if self.waypoint != 0:
                if self.human_detected:
                    self.stop_movement(5)
		    self.current_obstacle = "crosswalk"
		    self.update_obstacle_status()
                else:#사람이 있든 말든 횡단보도 보면 멈추라고 일단 해놈
                    self.stop_movement(5)
	    	    self.current_obstacle = "crosswalk"
		    self.update_obstacle_status()


##########################################################################
#opencv 신호등 구분
##########################################################################
    def color_callback(self, msg):
        color = msg.data
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    
        if self.current_obstacle == "stop":
            if color == "red" and not self.red_flag: # 빨간불 감지 및 플래그가 False일 경우
                self.stop_status_label.setText("빨간불 STOP")
                self.status_label.setText("빨간불 STOP")
                self.cmd_vel_pub.publish(twist)
                print("속도 0 publish")
            elif color == "green":
                self.red_flag = True
                self.status_label.setText("초록불 RESUME")
                self.resume_movement_2(12)	     	
                self.current_obstacle = "end"
                self.update_obstacle_status()
            elif color == "no":
                self.status_label.setText("색상 감지 준비중")

##########################################################################
#로봇 현재 위치 가져오기
#########################################################################
    def update_current_position(self, msg):
        self.current_position = msg.pose.pose

##########################################################################
#로봇 멈추기
##########################################################################
    def stop_movement(self, duration=None):
	if self.current_position is None:
	    print("현재 위치 정보가 없습니다.")
            return

        self.saved_waypoint = self.waypoint
	print("wp 저장")
	
	#self.move_command(self.current_position.position.x,
			  #self.current_position.position.y,
			  #self.current_position.orientation.z,
	                  #self.current_position.orientation.w)
	#print("제자리로 이동")

	#self.move_base_client.cancel_goal()
	#print("goal 삭제")

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
	print("속도 0")
	self.stop_status_label.setText("STOP")

	if duration is not None:
            start_time = rospy.get_time()
	    while rospy.get_time() - start_time < duration: # 과부하? 걸려서 10초 정도 지나면 찔끔찔끔 움직
	        if not rospy.is_shutdown():
	            self.cmd_vel_pub.publish(twist)
		    
		

	if self.current_obstacle == "crosswalk" or self.current_obstacle == "none": #몇 초 후 자동 출발임
	    self.resume_movement_2(12)

##########################################################################
#로봇 다시 이동
##########################################################################
    def resume_movement(self):
        if self.saved_waypoint is not None:
            print("정지 해제 :  {}로 이동 재개.".format(self.saved_waypoint))
	    self.stop_status_label.setText("Resume")


            self.navigation_mode = True
            self.choice_waypoint(self.saved_waypoint)  # 저장된 웨이포인트로 이동
            self.saved_waypoint = None  # 웨이포인트 이동 후 초기화
        else:
            print("저장된 웨이포인트가 없습니다.")

##########################################################################
#로봇 다시 이동(초록불)
##########################################################################
    def resume_movement_2(self, go_waypoint=None):
            print("정지 해제 :  {}로 이동 재개.".format(go_waypoint))
	    self.stop_status_label.setText("Resume")


            self.navigation_mode = True
            self.choice_waypoint(go_waypoint)  # 저장된 웨이포인트로 이동
            self.saved_waypoint = None  # 웨이포인트 이동 후 초기화

#####################################################################
#현재 장애물 변경
#####################################################################
    def change_traffic(self):
	self.current_obstacle = "traffic"
	self.update_obstacle_status()

#####################################################################
#현재 장애물 변경
#####################################################################
    def update_obstacle_status(self):
        obstacle = self.current_obstacle
        self.obstacle_status_label.setText("현재 장애물은 {} 입니다.".format(obstacle))

#####################################################################
#방 이동 함수
#####################################################################          
    def roomb(self):
	subprocess.call(['rosservice', 'call', '/move_base/clear_costmaps'])
        self.waypoint = 11
        self.choice_waypoint(self.waypoint)
        print("상태 : ", self.waypoint)

    def exit_process(self):
        print("프로세스를 종료합니다.")
        rospy.signal_shutdown("Shutting down")
        self.close()


#####################################################################
#메인 함수
#####################################################################
if __name__ == '__main__':
    rospy.init_node('video_subscriber_node', anonymous=True)
   
    # ROS 스핀을 별도의 스레드로 실행
    import threading
    ros_thread = threading.Thread(target=lambda: rospy.spin())
    ros_thread.start()

    app = QApplication(sys.argv)
    window = VideoSubscriber()
    window.show()
    sys.exit(app.exec_())

    # 프로그램 종료 시 ROS 스핀 정지
    ros_thread.join()
