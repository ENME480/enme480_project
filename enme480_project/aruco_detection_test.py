#!/usr/bin/env python
import cv2
import numpy as np
import cv2.aruco as aruco
import yaml

def detect_aruco(img, camera_matrix, distortion_coefficients):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the aruco markers and display its aruco id.

    if ids is not None:
        for i in range(len(ids)):
            # Draw the detected aruco onto the frame
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.0635, camera_matrix, distortion_coefficients)
            output = aruco.drawAxis(output, camera_matrix, distortion_coefficients, rvec, tvec, 0.1)
            
    return output, ids, corners

def get_aruco_center(img, ids, corners):
    if ids is not None:
        marker_centers = [(np.nan, np.nan)] * len(ids)
        for i in range(len(ids)):
            corner = corners[i][0, :, :]
            marker_centers[i] = np.mean(corner, axis=0)

            x = marker_centers[i][0]
            y = marker_centers[i][1]
            # Draw the detected marker center onto the frame with yellow color
            img = cv2.circle(img, (int(x), int(y)), 3, (0, 255, 255), -1)
    else:
        marker_centers = None
    return img, ids, marker_centers

def image_frame_to_table_frame(img, ids, marker_centers, perspective_matrix):
    if ids is not None:
        marker_centers_in_table_frame = [(np.nan, np.nan)] * len(ids)

        ############################ YOUR CODE STARTS HERE #####################################

        for i in range(len(ids)):




            # Assign final x and y coordinates after transformation

            marker_centers_in_table_frame = None

            # Format the coordinates as strings for display
            value1 = ""
            value2 = ""


            # Display the information on the image
            img = cv2.putText(img, "Aruco ID: " + str(ids[i]), (400, 20 + 90*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA, False) 
            img = cv2.putText(img, "X (mm): " + value1, (400, 50 + 90*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA, False) 
            img = cv2.putText(img, "Y (mm): " + value2, (400, 80 + 90*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA, False)
        ############################ YOUR CODE ENDS HERE #####################################
    else:
        marker_centers_in_table_frame = None 

    return img, marker_centers_in_table_frame


def get_camera_calibration_params(path):
    # Get the camera calibration parameters from the .yaml file
    with open(path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    cm = yaml_data['camera_matrix']['data']
    cm = np.array(cm)
    camera_matrix = cm.reshape(3,3)

    distortion_coefficients = yaml_data['distortion_coefficients']['data']
    distortion_coefficients = np.array(distortion_coefficients)
    distortion_coefficients = distortion_coefficients.reshape(1,5)

    return camera_matrix, distortion_coefficients


def GetStream(video_in, camera_matrix, distortion_coefficients, perspective_matrix):
    cap = cv2.VideoCapture(video_in)
    if not cap.isOpened():
        print("Error opening cam.")
        exit(0)

    rval, frame = cap.read()

    while rval:
        
        rval, frame = cap.read()
        key = cv2.waitKey(20)

        frame, ids, corners = detect_aruco(frame, camera_matrix, distortion_coefficients)
        frame, ids, marker_centers = get_aruco_center(frame, ids, corners)
        frame, marker_centers_in_table_frame = image_frame_to_table_frame(frame, ids, marker_centers, perspective_matrix)

        cv2.imshow("Stream: cam 0: ", frame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    if save_video_flag:
        out.release()
    cv2.destroyWindow("Stream: cam 0: ")

if __name__ == '__main__':

    # Load the Camera to Base Transformation Matrix
    perspective_matrix = np.load("/home/enme480_docker/enme480_ws/src/enme480_project/enme480_project/perspective_matrix.npy")
    path_to_camera_matrix = '/home/enme480_docker/enme480_ws/src/enme480_project/enme480_project/config/logitech_webcam_640x480.yaml'

    camera_matrix, distortion_coefficients = get_camera_calibration_params(path_to_camera_matrix)
    cam_0 = "/dev/video0"
    GetStream(cam_0, camera_matrix, distortion_coefficients, perspective_matrix)
    
