#!/usr/bin/env python

import sys
import copy
import time
import yaml

# import math libraries
import math
import numpy as np

# import CV libraries
import cv2
import cv2.aruco as aruco


# import ROS libraries
import rclpy
from rclpy.node import Node

# import ROS message libraries

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

# import custom messages and functions
from ur3e_mrc.msg import PositionUR3e, CommandUR3e, GripperInput

from enme480_project.kinematic_functions import KinematicFunctions
from enme480_project.block_detection_aruco import ArucoTracker 

KF = KinematicFunctions()
ik = KF.inverse_kinematics

#################### END OF IMPORT #########################

class UR3eController(Node):

    def __init__(self):
        super().__init__('ur3e_controller')
        self.home = [np.radians(85), np.radians(-45), np.radians(45), np.radians(-90), np.radians(-90), np.radians(90)]
        self.current_position = self.home
        self.thetas = [0.0] * 6
        self.gripper_toggle_state = False
        self.current_position_set = False
        self.digital_in_0 = 0
        self.SPIN_RATE = 20  # Adjust as needed
        self.vel = 0.1
        self.accel = 0.1

        # Publishers and Subscribers
        self.pub_command = self.create_publisher(CommandUR3e, 'ur3/command', 10)
        self.sub_position = self.create_subscription(PositionUR3e, 'ur3/position', self.position_callback, 10)
        self.sub_input = self.create_subscription(GripperInput, 'ur3/gripper_input', self.input_callback, 10)

        # Timer (if needed)
        # self.timer = self.create_timer(1.0 / self.SPIN_RATE, self.timer_callback)

    def position_callback(self, msg):
        self.thetas = msg.position
        self.current_position = list(self.thetas)
        self.current_position_set = True

    def input_callback(self, msg):
        self.digital_in_0 = msg.dig_in & 1  

    def move_arm(self, dest):

        '''
        CommandUR3e.msg:
                float64[] destination  ----> joint angles
                float64 v ----> velocity
                float64 a ----> acceleration
                bool io_0 ----> vacuum gripper input (True/False)
        '''

        ################## YOUR CODE STARTS HERE ##################################
        


        ################## YOUR CODE ENDS HERE ##################################

        # Wait until the robot reaches the goal
        while not self.at_goal(dest):
            rclpy.spin_once(self)
            time.sleep(0.05)  # Adjust sleep duration as needed
            pass

    def gripper_control(self, toggle_state):

        '''
        CommandUR3e.msg:
                float64[] destination  ----> joint angles
                float64 v ----> velocity
                float64 a ----> acceleration
                bool io_0 ----> vacuum gripper input (True/False)
        '''

        ################## YOUR CODE STARTS HERE ##################################



        ################## YOUR CODE ENDS HERE ##################################

        # Wait until the robot reaches the goal
        while not self.at_goal(self.current_position):
            rclpy.spin_once(self)
            time.sleep(0.05)  # Adjust sleep duration as needed
            pass

    def at_goal(self, destination):
        tolerance = 0.0005
        return all(abs(self.thetas[i] - destination[i]) < tolerance for i in range(6))


class BlockMover:

    def __init__(self, ur3e_controller, aruco_tracker, dest_positions):
        self.ur3e_controller = ur3e_controller
        self.aruco_tracker = aruco_tracker
        self.dest_positions = dest_positions
        self.intermediate_height = 0.2

    def move_block(self, initial_position, final_position):

        '''
        initial_position & final_position are lists ---> [x, y, z, yaw]

        TODO: Define the sequence for moving one single block from one position to the other
        '''

        ################## YOUR CODE STARTS HERE #######################################


        ################## YOUR CODE ENDS HERE #######################################

    def process_blocks(self, video_device, ids, destination):

        '''
        This function is used for processing the Aruco Markers, find their centers and convert them to table frame. Once that's done, the function decides the sequence of block picking and end destination for each block.

        TODO: Detect the aruco tags, find their centers, and convert the center position to image frame. 
              Then, strategize on how you want to move the blocks (eg. different groups, single stack, grouped stacks). You can include all of them in this function if you want to try multiple methods. 
              
              You can use the move_block function defined above. Before moving the blocks, send a command to move the robot to home position defined in the UR3eController class

            Below are the corresponding Aruco IDs for each color in RAL
                    block_color = Yellow    - id: 100
                    block_color = Red       - id: 150
                    block_color = Blue      - id: 200

        '''

        frame = self.aruco_tracker.get_frame(video_device)

        ################## YOUR CODE STARTS HERE ##################################


        ################## YOUR CODE ENDS HERE ##################################

def main():

    rclpy.init()

    try:

        # # Create an instance of UR3eController
        ur3e_controller = UR3eController()

        aruco_tracker = ArucoTracker(
            camera_matrix_path='/home/enme480_docker/ENME480_ws/src/enme480_project/enme480_project/config/logitech_webcam_640x480.yaml',
            perspective_matrix_path='/home/enme480_docker/ENME480_ws/src/enme480_project/enme480_project/perspective_matrix.npy'
        )

        # Define destination positions for each block. These are recommended sorting positions for different colors. The given z is the lowest level in the stack
        dest_pose = [
            (0.1, -0.1, 0.045, 0), # Yellow Drop Off 
            (0.2, -0.1, 0.045, 0), # Red Drop Off
            (0.3, -0.1, 0.045, 0), # Blue Drop Off
        ]

        # Initialize BlockMover and start moving blocks
        block_mover = BlockMover(ur3e_controller, aruco_tracker, dest_pose)
        block_ids = [100, 150, 200]  # Block IDs to move
        block_mover.process_blocks("/dev/video0", block_ids, dest_pose)


    except KeyboardInterrupt:

        ur3e_controller.get_logger().info("Test interrupted by user.")

    finally:

        # Clean up and shut down
        ur3e_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
