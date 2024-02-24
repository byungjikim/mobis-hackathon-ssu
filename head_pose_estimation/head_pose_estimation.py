"""
Created on Fri Jul 31 03:00:36 2020

@author: hp
"""
import sys
import os
curr_dir = "/home/user/head_pose_ros2/src/head_pose_estimation/head_pose_estimation"
os.chdir(curr_dir)
sys.path.append(curr_dir)

import cv2
import numpy as np
import math
import rclpy
import tensorflow as tf
from tensorflow import keras
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from face_detector import get_face_detector, find_faces
from face_landmarks import get_landmark_model, detect_marks
from threading import Lock


class HeadPosePub(Node):
    def __init__(self):
        super().__init__('head_pose_publisher')
        self.publisher = self.create_publisher(String, '/head_pose', 10)
        self.subscriber = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.timer = self.create_timer(0, self.timer_callback)
        self.bridge = CvBridge()
        self.frame = None
        self.succeed_call = False
        self.lock = Lock()
        self.size = 0
        self.model_points = np.array([
                                (0.0, 0.0, 0.0),             # Nose tip
                                (0.0, -330.0, -65.0),        # Chin
                                (-225.0, 170.0, -135.0),     # Left eye left corner
                                (225.0, 170.0, -135.0),      # Right eye right corne
                                (-150.0, -150.0, -125.0),    # Left Mouth corner
                                (150.0, -150.0, -125.0)      # Right mouth corner
                            ])
        self.face_model = get_face_detector()
        self.landmark_model = get_landmark_model()

    def publish_headPose(self, pose):
        msg = String()
        msg.data = pose
        self.publisher.publish(msg)

    def image_callback(self, msg):
        with self.lock:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.succeed_call = True
            if(self.size ==0):
                self.size = self.frame.shape
                focal_length = self.size[1]
                center = (self.size[1]/2, self.size[0]/2)
                self.camera_matrix = np.array(
                                        [[focal_length, 0, center[0]],
                                        [0, focal_length, center[1]],
                                        [0, 0, 1]], dtype = "double"
                                        )


    def timer_callback(self):
        if(self.succeed_call):
            with self.lock:
                img = self.frame
                faces = find_faces(img, self.face_model)
                for face in faces:
                    marks = detect_marks(img, self.landmark_model, face)
                    # marks = detect_marks(frame, self.landmark_model, face)

                    # mark_detector.draw_marks(img, marks, color=(0, 255, 0))
                    image_points = np.array([
                                            marks[30],     # Nose tip
                                            marks[8],     # Chin
                                            marks[36],     # Left eye left corner
                                            marks[45],     # Right eye right corne
                                            marks[48],     # Left Mouth corner
                                            marks[54]      # Right mouth corner
                                        ], dtype="double")
                    dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
                    (success, rotation_vector, translation_vector) = cv2.solvePnP(self.model_points, image_points, self.camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_UPNP)
                    
                    # Project a 3D point (0, 0, 1000.0) onto the image plane.
                    # We use this to draw a line sticking out of the nose
                    
                    (nose_end_point2D, jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector, translation_vector, self.camera_matrix, dist_coeffs)
                                            
                    p1 = ( int(image_points[0][0]), int(image_points[0][1]))
                    p2 = ( int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
                    x1, x2 = head_pose_points(img, rotation_vector, translation_vector, self.camera_matrix)
                    # x1, x2 = head_pose_points(frame, rotation_vector, translation_vector, camera_matrix)

                    
                    try:
                        m = (p2[1] - p1[1])/(p2[0] - p1[0])
                        ang1 = int(math.degrees(math.atan(m)))
                    except:
                        ang1 = 90
                        
                    try:
                        m = (x2[1] - x1[1])/(x2[0] - x1[0])
                        ang2 = int(math.degrees(math.atan(-1/m)))
                    except:
                        ang2 = 90
                        
                    head_pose_estimation = detect_head_pose(ang1, ang2)
                    # head_pose_pub.publish_headPose(head_pose_estimation)
                    self.publish_headPose(head_pose_estimation)
                    self.succeed_call = False
        



def get_2d_points(img, rotation_vector, translation_vector, camera_matrix, val):
    """Return the 3D points present as 2D for making annotation box"""
    point_3d = []
    dist_coeffs = np.zeros((4,1))
    rear_size = val[0]
    rear_depth = val[1]
    point_3d.append((-rear_size, -rear_size, rear_depth))
    point_3d.append((-rear_size, rear_size, rear_depth))
    point_3d.append((rear_size, rear_size, rear_depth))
    point_3d.append((rear_size, -rear_size, rear_depth))
    point_3d.append((-rear_size, -rear_size, rear_depth))
    
    front_size = val[2]
    front_depth = val[3]
    point_3d.append((-front_size, -front_size, front_depth))
    point_3d.append((-front_size, front_size, front_depth))
    point_3d.append((front_size, front_size, front_depth))
    point_3d.append((front_size, -front_size, front_depth))
    point_3d.append((-front_size, -front_size, front_depth))
    point_3d = np.array(point_3d, dtype=np.float64).reshape(-1, 3)
    
    # Map to 2d img points
    (point_2d, _) = cv2.projectPoints(point_3d,
                                      rotation_vector,
                                      translation_vector,
                                      camera_matrix,
                                      dist_coeffs)
    point_2d = np.int32(point_2d.reshape(-1, 2))
    return point_2d

    
    
def head_pose_points(img, rotation_vector, translation_vector, camera_matrix):
    """
    Get the points to estimate head pose sideways    

    Parameters
    ----------
    img : np.unit8
        Original Image.
    rotation_vector : Array of float64
        Rotation Vector obtained from cv2.solvePnP
    translation_vector : Array of float64
        Translation Vector obtained from cv2.solvePnP
    camera_matrix : Array of float64
        The camera matrix

    Returns
    -------
    (x, y) : tuple
        Coordinates of line to estimate head pose

    """
    rear_size = 1
    rear_depth = 0
    front_size = img.shape[1]
    front_depth = front_size*2
    val = [rear_size, rear_depth, front_size, front_depth]
    point_2d = get_2d_points(img, rotation_vector, translation_vector, camera_matrix, val)
    y = (point_2d[5] + point_2d[8])//2
    x = point_2d[2]
    
    return (x, y)

def detect_head_pose(ang1, ang2):
    direction = ""
    if ang1 >= 48:
        direction += 'Head down'
    elif ang1 <= -48:
        direction += 'Head up'
        
    if ang2 >= 48:
        direction += 'Head right'
    elif ang2 <= -48:
        direction += 'Head left'
    
    return direction

def main(args=None):
    rclpy.init(args=args)
    head_pose_pub = HeadPosePub()
    rclpy.spin(head_pose_pub)
    # face_model = get_face_detector()
    # landmark_model = get_landmark_model()
    # cap = cv2.VideoCapture(0)
    # ret, img = cap.read()
    # size = img.shape

    # font = cv2.FONT_HERSHEY_SIMPLEX 
    # # 3D model points.
    # model_points = np.array([
    #                             (0.0, 0.0, 0.0),             # Nose tip
    #                             (0.0, -330.0, -65.0),        # Chin
    #                             (-225.0, 170.0, -135.0),     # Left eye left corner
    #                             (225.0, 170.0, -135.0),      # Right eye right corne
    #                             (-150.0, -150.0, -125.0),    # Left Mouth corner
    #                             (150.0, -150.0, -125.0)      # Right mouth corner
    #                         ])

    # # Camera internals
    # focal_length = size[1]
    # center = (size[1]/2, size[0]/2)
    # camera_matrix = np.array(
    #                         [[focal_length, 0, center[0]],
    #                         [0, focal_length, center[1]],
    #                         [0, 0, 1]], dtype = "double"
    #                         )
    
    # while True:
    #     ret, img = cap.read()
    #     if ret == True:
    #         faces = find_faces(img, face_model)
    #         for face in faces:
    #             # marks = detect_marks(img, landmark_model, face)
    #             marks = detect_marks(frame, landmark_model, face)

    #             # mark_detector.draw_marks(img, marks, color=(0, 255, 0))
    #             image_points = np.array([
    #                                     marks[30],     # Nose tip
    #                                     marks[8],     # Chin
    #                                     marks[36],     # Left eye left corner
    #                                     marks[45],     # Right eye right corne
    #                                     marks[48],     # Left Mouth corner
    #                                     marks[54]      # Right mouth corner
    #                                 ], dtype="double")
    #             dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
    #             (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_UPNP)
                
    #             # Project a 3D point (0, 0, 1000.0) onto the image plane.
    #             # We use this to draw a line sticking out of the nose
                
    #             (nose_end_point2D, jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
                                        
    #             p1 = ( int(image_points[0][0]), int(image_points[0][1]))
    #             p2 = ( int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
    #             # x1, x2 = head_pose_points(img, rotation_vector, translation_vector, camera_matrix)
    #             x1, x2 = head_pose_points(frame, rotation_vector, translation_vector, camera_matrix)

                
    #             try:
    #                 m = (p2[1] - p1[1])/(p2[0] - p1[0])
    #                 ang1 = int(math.degrees(math.atan(m)))
    #             except:
    #                 ang1 = 90
                    
    #             try:
    #                 m = (x2[1] - x1[1])/(x2[0] - x1[0])
    #                 ang2 = int(math.degrees(math.atan(-1/m)))
    #             except:
    #                 ang2 = 90
                    
    #             head_pose_estimation = detect_head_pose(ang1, ang2)
    #             head_pose_pub.publish_headPose(head_pose_estimation)

    #         # if cv2.waitKey(1) & 0xFF == ord('q'):
    #         #     break

    #     else:
    #         break
    # cap.release()
    # cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()