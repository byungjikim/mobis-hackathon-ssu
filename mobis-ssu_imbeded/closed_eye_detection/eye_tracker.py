# -*- coding: utf-8 -*-
"""
Created on Thu Jul 30 19:21:18 2020

@author: hp
"""
import sys
import os
curr_dir = "/home/user/eye_tracking_ros2/src/eye_tracker/eye_tracker"
sys.path.append(curr_dir)
os.chdir(curr_dir)
import cv2
import numpy as np
import rclpy

from face_detector import get_face_detector, find_faces
from face_landmarks import get_landmark_model, detect_marks
from std_msgs.msg import String
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Lock

class closedEyeTrackingPub(Node):
    def __init__(self):
        super().__init__('eye_closed_publisher')
        self.publisher_ = self.create_publisher(String, '/eye_position', 10)
        self.subscriber = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.timer = self.create_timer(0, self.timer_callback)
        self.bridge = CvBridge()
        self.lock = Lock()
        self.face_model = get_face_detector()
        self.landmark_model = get_landmark_model()
        self.left = [36, 37, 38, 39, 40, 41]
        self.right = [42, 43, 44, 45, 46, 47]
        self.succeed_call = False
        self.frame = None
        self.kernel = np.ones((9, 9), np.uint8)
        self.threshold = 115
        self.dark_color_threshold = 20
        # self.eyes = None
        # cv2.namedWindow("Masked Eyes")
        # cv2.namedWindow("image")

    def publish_eye_position(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)

    def image_callback(self, msg):
        with self.lock:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.succeed_call = True

    def timer_callback(self):
        if(self.succeed_call):
            with self.lock:
                img = self.frame
                rects = find_faces(img, self.face_model)
                for rect in rects:
                    shape = detect_marks(img, self.landmark_model, rect)
                    mask = np.zeros(img.shape[:2], dtype=np.uint8)
                    mask, end_points_left = eye_on_mask(mask, self.left, shape)
                    mask, end_points_right = eye_on_mask(mask, self.right, shape)
                    mask = cv2.dilate(mask, self.kernel, 5)

                    eyes = cv2.bitwise_and(img, img, mask=mask)
                    dark_pixels_mask = cv2.cvtColor(eyes, cv2.COLOR_BGR2GRAY) < self.dark_color_threshold
                    eyes[dark_pixels_mask] = [255, 255, 255]

                    mid = int((shape[42][0] + shape[39][0]) // 2)
                    eyes_gray = cv2.cvtColor(eyes, cv2.COLOR_BGR2GRAY)

                    _, thresh = cv2.threshold(eyes_gray, self.threshold, 255, cv2.THRESH_BINARY)
                    thresh = process_thresh(thresh)
                    
                    eyeball_pos_left = contouring(thresh[:, 0:mid], mid, img, end_points_left)
                    eyeball_pos_right = contouring(thresh[:, mid:], mid, img, end_points_right, True)
                    left_close = cal_pupil_ratio(thresh, end_points_left, self.dark_color_threshold)
                    right_close = cal_pupil_ratio(thresh, end_points_right, self.dark_color_threshold)

                    self.publish_eye_position(judge_closedEye(left_close, right_close))
                    self.succeed_call = False
                # cv2.imshow('Processed Image', img)
                # cv2.imshow('thresh', thresh)


    def nothing(self,x):
        pass

def eye_on_mask(mask, side, shape):
    """
    Create ROI on mask of the size of eyes and also find the extreme points of each eye

    Parameters
    ----------
    mask : np.uint8
        Blank mask to draw eyes on
    side : list of int
        the facial landmark numbers of eyes
    shape : Array of uint32
        Facial landmarks

    Returns
    -------
    mask : np.uint8
        Mask with region of interest drawn
    [l, t, r, b] : list
        left, top, right, and bottommost points of ROI

    """
    points = [shape[i] for i in side]
    points = np.array(points, dtype=np.int32)
    mask = cv2.fillConvexPoly(mask, points, 255)
    l = points[0][0]
    t = (points[1][1]+points[2][1])//2
    r = points[3][0]
    b = (points[4][1]+points[5][1])//2
    return mask, [l, t, r, b]

def find_eyeball_position(end_points, cx, cy):
    """Find and return the eyeball positions, i.e. left or right or top or normal"""
    x_ratio = (end_points[0] - cx)/(cx - end_points[2])
    y_ratio = (cy - end_points[1])/(end_points[3] - cy)
    if x_ratio > 3:
        return 1
    elif x_ratio < 0.33:
        return 2
    elif y_ratio < 0.33:
        return 3
    else:
        return 0

    
def contouring(thresh, mid, img, end_points, right=False):
    """
    Find the largest contour on an image divided by a midpoint and subsequently the eye position

    Parameters
    ----------
    thresh : Array of uint8
        Thresholded image of one side containing the eyeball
    mid : int
        The mid point between the eyes
    img : Array of uint8
        Original Image
    end_points : list
        List containing the exteme points of eye
    right : boolean, optional
        Whether calculating for right eye or left eye. The default is False.

    Returns
    -------
    pos: int
        the position where eyeball is:
            0 for normal
            1 for left
            2 for right
            3 for up

    """
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    try:
        cnt = max(cnts, key = cv2.contourArea)
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        if right:
            cx += mid
        # cv2.circle(img, (cx, cy), 4, (0, 0, 255), 2)
        pos = find_eyeball_position(end_points, cx, cy)
        return pos
    except:
        pass
    
def process_thresh(thresh):
    """
    Preprocessing the thresholded image

    Parameters
    ----------
    thresh : Array of uint8
        Thresholded image to preprocess

    Returns
    -------
    thresh : Array of uint8
        Processed thresholded image

    """
    thresh = cv2.erode(thresh, None, iterations=2) 
    thresh = cv2.dilate(thresh, None, iterations=4) 
    thresh = cv2.medianBlur(thresh, 3) 
    thresh = cv2.bitwise_not(thresh)
    return thresh

def estimate_eye_pos(img, left, right):
    """
    Print the side where eye is looking and display on image

    Parameters
    ----------
    img : Array of uint8
        Image to display on
    left : int
        Position obtained of left eye.
    right : int
        Position obtained of right eye.

    Returns
    -------
    None.

    """
    text = 'None'

    if left == right and left != 0:
        if left == 1:
            # print('Looking left')
            text = 'Looking left'
        elif left == 2:
            # print('Looking right')
            text = 'Looking right'
        elif left == 3:
            # print('Looking up')
            text = 'Looking up'
        
    return text

def cal_pupil_ratio(thresh_img, roi, dark_threashold):
    left, top, right, bottom = roi
    roi_img = thresh_img[top:bottom, left:right]

    pupil_pixel = roi_img < dark_threashold
    total_pupil_pixels = np.sum(pupil_pixel)

    total_pixel_count = roi_img.size

    pupil_pixel_ratio = total_pupil_pixels / total_pixel_count

    if(pupil_pixel_ratio < 0.3):
        return True
    
    return False

def judge_closedEye(left_bool, right_bool):
    if (left_bool and right_bool):
        return "True"
    
    return "False"

def main(args=None):
    rclpy.init(args=args)
    eye_closed_publisher = closedEyeTrackingPub()
    rclpy.spin(eye_closed_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()