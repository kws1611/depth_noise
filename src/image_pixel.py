#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

# ROS 이미지 메시지 수신 콜백 함수
def image_callback(msg):
    # CvBridge 객체 생성
    bridge = CvBridge()
    # ROS 이미지 메시지를 OpenCV 이미지로 변환
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    # OpenCV 이미지 출력
    cv2.imshow("ROS Depth Image", depth_image)
    cv2.waitKey(1)

    # 마우스 클릭 이벤트 핸들러 함수
    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 마우스 왼쪽 버튼 클릭 시 선택한 위치의 픽셀 값 추출
            depth_pixel = depth_image[y, x]
            print("Depth value at ({}, {}): {}".format(x, y, depth_pixel))

    # 영상 출력 및 마우스 클릭 이벤트 핸들러 등록
    cv2.namedWindow("ROS Depth Image", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("ROS Depth Image", click_event)

# ROS 노드 초기화 및 이미지 메시지 수신자 등록
rospy.init_node("depth_image_subscriber")
rospy.Subscriber("/camera/depth/image_rect_raw", Image, image_callback)

# ROS 메시지 수신 대기
rospy.spin()

# OpenCV 윈도우 종료
cv2.destroyAllWindows()
