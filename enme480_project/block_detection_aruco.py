#!/usr/bin/env python3
import cv2
import numpy as np
import cv2.aruco as aruco
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String  # For publishing positions as text

class ArucoTracker(Node):
    def __init__(self, camera_matrix_path, perspective_matrix_path):
        super().__init__('aruco_tracker')
        
        # Load camera calibration parameters and perspective transformation matrix
        self.camera_matrix, self.distortion_coefficients = self.load_camera_calibration(camera_matrix_path)
        self.perspective_matrix = np.load(perspective_matrix_path)
        
        # Initialize ROS2 subscribers, publishers, and CvBridge
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(Image, '/camera1/image_raw', self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/aruco_detection/image', 10)
        self.position_publisher = self.create_publisher(String, '/aruco_detection/positions', 10)

    @staticmethod
    def crop_frame(image):
        margin_up, margin_down, margin_left, margin_right = 230, 0, 60, 52
        h, w, _ = image.shape
        return image[margin_up:h-margin_down, margin_left:w-margin_right]

    def get_frame(self,video_in):
        cap = cv2.VideoCapture(video_in)
        if not cap.isOpened():
            print("Error opening cam.")
            exit(0)
        '''
        #cap.set(1, k_frame)
        rval, frame = cap.read()
        frame_filename = ""
        frame = CropFrame(frame)
        cv2.imwrite(frame_filename, frame)
        print("Obtaining current frame.")
        '''
        rval, frame = cap.read()

        print("Obtaining current frame.")

        return frame

    def load_camera_calibration(self, path):
        with open(path, 'r') as file:
            yaml_data = yaml.safe_load(file)
        cm = np.array(yaml_data['camera_matrix']['data']).reshape(3, 3)
        distortion_coefficients = np.array(yaml_data['distortion_coefficients']['data']).reshape(1, 5)
        return cm, distortion_coefficients

    def detect_aruco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        output = aruco.drawDetectedMarkers(img, corners, ids)

        if ids is not None:
            for i in range(len(ids)):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.0635, self.camera_matrix, self.distortion_coefficients)
                output = aruco.drawAxis(output, self.camera_matrix, self.distortion_coefficients, rvec, tvec, 0.1)

        return output, ids, corners

    def get_aruco_center(self, img, ids, corners):
        marker_centers = [(np.nan, np.nan)] * len(ids)
        for i in range(len(ids)):
            corner = corners[i][0, :, :]
            marker_centers[i] = np.mean(corner, axis=0)
            x, y = marker_centers[i]
            img = cv2.circle(img, (int(x), int(y)), 3, (0, 255, 255), -1)
        return img, ids, marker_centers

    def image_frame_to_table_frame(self, img, ids, marker_centers, perspective_matrix):
        marker_centers_in_table_frame = [(np.nan, np.nan)] * len(ids)

        ################################### YOUR CODE STARTS HERE ##################################################

        # Format the coordinates as strings for display
    
        # Display the information on the image
        img = cv2.putText(img, "Aruco ID: " + str(ids[i]), (400, 20 + 90*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA, False) 
        img = cv2.putText(img, "X (mm): " + value1, (400, 50 + 90*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA, False) 
        img = cv2.putText(img, "Y (mm): " + value2, (400, 80 + 90*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA, False)
        ################################### YOUR CODE ENDS HERE ####################################################
        
        return img, marker_centers_in_table_frame

    def image_callback(self, msg):
        # Convert ROS2 image message to OpenCV format
        # print(msg)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the frame
        frame, ids, corners = self.detect_aruco(frame)
        if ids is not None:
            frame, ids, marker_centers = self.get_aruco_center(frame, ids, corners)
            frame, _ = self.image_to_table_frame(frame, ids, marker_centers)

        # Publish the processed image
        processed_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(processed_image_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_matrix_path = '/home/enme480_docker/ENME480_ws/src/enme480_project/enme480_project/config/logitech_webcam_640x480.yaml'
    perspective_matrix_path = '/home/enme480_docker/ENME480_ws/src/enme480_project/enme480_project/perspective_matrix.npy'
    
    # Initialize and spin the Aruco tracker node
    tracker = ArucoTracker(camera_matrix_path, perspective_matrix_path)
    rclpy.spin(tracker)

    # Cleanup
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
