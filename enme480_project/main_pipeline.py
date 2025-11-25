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

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

# import custom messages and functions
from ur3e_mrc_msgs.msg import PositionUR3e, CommandUR3e, GripperInput

from enme480_project.kinematic_functions import KinematicFunctions
from enme480_project.block_detection_aruco import ArucoTracker 

KF = KinematicFunctions()
ik = KF.inverse_kinematics

# Global list of nodes to spin
NODES = []

def spin_all(timeout=0.05):
    """
    Spin all registered ROS2 nodes once with a given timeout.

    This helper keeps the example single-threaded by calling `rclpy.spin_once`
    on every node in the global `NODES` list. It is used throughout the script
    anywhere we need callbacks (e.g., joint state updates, camera images)
    to be processed without starting a full blocking spin loop.
    """
    for node in NODES:
        rclpy.spin_once(node, timeout_sec=timeout)


#################### END OF IMPORT #########################

class UR3eController(Node):

    def __init__(self):
        """
        Initialize the UR3e controller node and create ROS2 I/O.

        - Sets up default home joint configuration and internal state used
          to track the robot's current joint positions and gripper input.
        - Creates publishers and subscribers for commanding the UR3e arm
          (`ur3e/command`), reading joint positions (`ur3e/position`),
          and reading the gripper vacuum input (`/gripper/vac_on`).
        - You should not need to modify this constructor for basic pick-and-place
          behavior; your logic will typically go in the methods below.
        """
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
        self.pub_command = self.create_publisher(CommandUR3e, 'ur3e/command', 10)
        self.sub_position = self.create_subscription(PositionUR3e, 'ur3e/position', self.position_callback, 10)
        self.sub_input = self.create_subscription(Bool, '/gripper/vac_on', self.input_callback, 10)

        # Timer (if needed)
        # self.timer = self.create_timer(1.0 / self.SPIN_RATE, self.timer_callback)

    def position_callback(self, msg):
        """
        ROS2 subscription callback for joint positions.

        Each time a `PositionUR3e` message arrives, this updates the internal
        joint vector (`self.thetas` and `self.current_position`) and sets
        `self.current_position_set` to indicate that at least one valid joint
        state has been received from the real or simulated robot.
        """
        self.thetas = msg.position
        self.current_position = list(self.thetas)
        self.current_position_set = True

    def input_callback(self, msg):
        """
        ROS2 subscription callback for the vacuum gripper digital input.

        The incoming message indicates whether the gripper suction is detected
        as on or off. This method maps that information to a simple integer
        flag (`self.digital_in_0`) that can be used by higher-level logic
        to confirm that a grasp has succeeded or that a block has been released.
        """
        if msg.data == True:
            self.digital_in_0 = 1
        else:
            self.digital_in_0 = 0
        # self.digital_in_0 = msg.dig_in & 1  

    def move_arm(self, dest, timeout=10.0):

        """
        Command the UR3e arm to move to a joint-space goal.

        - `dest` is a list of 6 joint angles (in radians) representing the
          desired target pose for the robot.
        - Inside the YOUR CODE section you must create and publish a
          `CommandUR3e` message that sets:
              - `destination` to the `dest` list,
              - `v` and `a` to the velocity and acceleration stored in
                `self.vel` and `self.accel`,
              - `io_0` to the current gripper state (do not change the gripper
                here; just preserve its current on/off value).
        - After publishing the command, the loop below waits until the robot
          has reached the goal, or until `timeout` seconds have elapsed.
        """

        # CommandUR3e.msg:
        #         float64[] destination  ----> joint angles
        #         float64 v ----> velocity
        #         float64 a ----> acceleration
        #         bool io_0 ----> vacuum gripper input (True/False)

        ################## YOUR CODE STARTS HERE ##################################
        # EXPECTATION: Construct a CommandUR3e message for a pure arm move.
        #
        # Typical steps to implement here:
        #   1. Create an instance of CommandUR3e.
        #   2. Set `msg.destination` to the provided `dest` joint list.
        #   3. Set `msg.v` and `msg.a` using `self.vel` and `self.accel`,
        #      so that all arm moves are executed with the configured speed.
        #   4. Set `msg.io_0` to reflect the current gripper state
        #      (e.g., based on `self.gripper_toggle_state` or similar logic),
        #      but do NOT toggle the gripper here.
        #   5. Publish the message on `self.pub_command` to send it to the
        #      UR3e control interface.
        #
        # Do not modify the waiting loop below; it already waits for the
        # robot to reach the desired destination using `self.at_goal`.
        


        ################## YOUR CODE ENDS HERE ##################################

        # Wait until the robot reaches the goal
        start = time.time()
        while rclpy.ok() and not self.at_goal(dest):
            if time.time() - start > timeout:
                err = self.goal_error(dest)
                self.get_logger().warn(
                    f"Timeout waiting for arm to reach destination. "
                    f"Joint errors: {['%.4f' % e for e in err]}"
                )
                break
            spin_all(0.05)

    def gripper_control(self, toggle_state, timeout=5.0):

        """
        Command the UR3e gripper on or off while holding the current pose.

        - `toggle_state` is a boolean indicating the desired gripper vacuum
          state (`True` for suction on, `False` for suction off).
        - Inside the YOUR CODE section you must create and publish a
          `CommandUR3e` message that:
              - Keeps `destination` equal to the robot's current joint position
                (i.e., no motion in joint space).
              - Uses the standard velocity and acceleration (`self.vel`,
                `self.accel`) to maintain consistency with other commands.
              - Sets `io_0` according to `toggle_state` so the gripper is
                actually toggled.
        - The loop below waits until the command has been applied or until
          `timeout` seconds have elapsed.
        """

        # CommandUR3e.msg:
        #         float64[] destination  ----> joint angles
        #         float64 v ----> velocity
        #         float64 a ----> acceleration
        #         bool io_0 ----> vacuum gripper input (True/False)

        ################## YOUR CODE STARTS HERE ##################################
        # EXPECTATION: Construct a CommandUR3e message to toggle the gripper.
        #
        # Typical steps to implement here:
        #   1. Create an instance of CommandUR3e.
        #   2. Set `msg.destination` to `self.current_position` so the robot
        #      holds its current joint configuration.
        #   3. Set `msg.v` and `msg.a` using `self.vel` and `self.accel`
        #      (even though no motion is desired, these fields must be valid).
        #   4. Set `msg.io_0` based on the `toggle_state` argument
        #      (True -> vacuum on, False -> vacuum off).
        #   5. Update any internal gripper state variables if needed
        #      (e.g., `self.gripper_toggle_state`) so the rest of the system
        #      knows the intended gripper command.
        #   6. Publish the message on `self.pub_command`.
        #
        # The waiting loop below checks `self.at_goal(self.current_position)`
        # to allow time for the gripper state to take effect.



        ################## YOUR CODE ENDS HERE ##################################

        # Wait until the robot reaches the goal
        start = time.time()
        while rclpy.ok() and not self.at_goal(self.current_position):
            if time.time() - start > timeout:
                self.get_logger().warn("Timeout waiting for gripper command to apply.")
                break
            spin_all(0.05)

    def at_goal(self, destination, tolerance=0.0008):
        """
        Check whether the current joint angles are within a tolerance of a goal.

        - `destination` is a list of 6 desired joint values in radians.
        - `tolerance` defines the maximum absolute difference (per joint) that
          is allowed for the robot to be considered at the goal.
        - Returns `True` if all joint errors are below `tolerance`, otherwise
          returns `False`. This helper is used by move and gripper commands.
        """
        return all(abs(self.thetas[i] - destination[i]) < tolerance for i in range(6))

    def goal_error(self, destination):
        """
        Compute the absolute joint-wise error between current and desired poses.

        - `destination` is a list of 6 desired joint values in radians.
        - Returns a list of 6 absolute errors, one for each joint, which can
          be logged or inspected when movement timeouts occur.
        """
        return [abs(self.thetas[i] - destination[i]) for i in range(6)]


class BlockMover:

    def __init__(self, ur3e_controller, aruco_tracker, dest_positions):
        """
        Helper class that coordinates picking and placing blocks using the UR3e.

        - Holds references to the `UR3eController` (for motion and gripper
          commands) and the `ArucoTracker` (for block detections and positions).
        - `dest_positions` is a list of table-frame target poses (x, y, z, yaw)
          that define where different colored blocks should be dropped off.
        - `intermediate_height` defines the safe Z height used when moving
          above blocks to avoid collisions with the table or other blocks.
        """
        self.ur3e_controller = ur3e_controller
        self.aruco_tracker = aruco_tracker
        self.dest_positions = dest_positions
        self.intermediate_height = 0.2

    def move_block(self, initial_position, final_position):

        """
        Move a single block from an initial pose to a final pose.

        - `initial_position` and `final_position` are each lists of the form
          `[x, y, z, yaw]` in the table frame.
        - Inside the YOUR CODE section you must define a complete pick-and-place
          sequence that uses inverse kinematics (via `ik`) and the controller
          methods (`move_arm`, `gripper_control`) to:
              1. Move above the block at `initial_position` using a safe Z.
              2. Move down to grasp height and turn the gripper on.
              3. Lift the block back to a safe height.
              4. Move above the `final_position` and lower the block.
              5. Turn the gripper off and retreat to a safe height.
        - This function is called by `process_blocks` once the source and
          destination poses for each block are known.
        """

        ################## YOUR CODE STARTS HERE #######################################
        # EXPECTATION: Implement an end-to-end pick-and-place sequence.
        #
        # A typical implementation will:
        #   1. Use the provided `ik` function to convert the table-frame
        #      poses (`initial_position` and `final_position`) into joint
        #      configurations for the UR3e.
        #   2. Call `self.ur3e_controller.move_arm(...)` to move to:
        #        - A safe approach height above the initial block,
        #        - A lower height to grip the block,
        #        - A lift height after gripping,
        #        - A safe approach height above the final location,
        #        - A lower height to drop the block,
        #        - A lift or retreat height after release.
        #   3. Call `self.ur3e_controller.gripper_control(True/False)` at the
        #      right times to turn suction on for pickup and off for release.
        #   4. Respect `self.intermediate_height` as a safe Z offset to avoid
        #      collisions with the environment.
        #
        # All motion planning, sequencing, and choice of heights are left for
        # you to design within this block.


        ################## YOUR CODE ENDS HERE #######################################

    def process_blocks(self, destination):

        """
        Detect blocks with ArUco markers and decide how to move them.

        High-level responsibilities:
        - Wait for the `ArucoTracker` node to report at least one detection.
        - Use the tracker’s latest IDs and positions (in table frame) to
          identify where blocks of each color are currently located.
        - Decide on a block-moving strategy (e.g., group blocks by color,
          stack them, or arrange them in multiple stacks) and map each block
          to a destination pose from the `destination` list.
        - Use `BlockMover.move_block` to perform the actual pick-and-place
          for each block. Before starting, you should typically move the robot
          to the home joint configuration defined in `UR3eController`.

        TODO guidance:
        - Inside the YOUR CODE section later in this function, you will:
            1. Interpret `temp_ids` and `marker_positions` produced by the
               Aruco tracker (IDs: 100 = Yellow, 150 = Red, 200 = Blue).
            2. Convert those 2D positions into full poses `[x, y, z, yaw]`
               in the table frame using any fixed assumptions you need for
               `z` height and yaw.
            3. Decide which pose in `destination` each block should be moved
               to and call `self.move_block(initial_pose, final_pose)` for
               each block in your chosen sequence.
        """

        # Spin the ArucoTracker node until we have at least one detection
        self.aruco_tracker.get_logger().info("Waiting for ArUco detections from /camera...")
        start = time.time()

        while rclpy.ok() and self.aruco_tracker.latest_ids is None:
            if time.time() - start > 10.0:
                print("Timed out waiting for ArUco detections.")
                return
            spin_all(0.1)

        temp_ids = self.aruco_tracker.latest_ids
        marker_positions = self.aruco_tracker.latest_positions

        if temp_ids is None or len(temp_ids) == 0:
            print("No ArUco markers detected, aborting.")
            return

        ################## YOUR CODE STARTS HERE ##################################
        # EXPECTATION: Use detections to plan and execute block moves.
        #
        # Typical steps to implement here:
        #   1. Move the UR3e to its home configuration using IK + move_arm.
        #   2. From `temp_ids` and `marker_positions`, build a list of
        #      `(aruco_id, (x, y))` pairs; the positions are already in the
        #      table frame as provided by `ArucoTracker`.
        #   3. For each detected marker:
        #        - Infer the corresponding block color from its ID:
        #             100 -> Yellow, 150 -> Red, 200 -> Blue.
        #        - Create a full initial pose `[x, y, z, yaw]` for the block,
        #          where `z` and `yaw` can be fixed constants that match your
        #          table height and desired block orientation.
        #        - Choose an appropriate final pose from the `destination`
        #          list (e.g., first index for Yellow, second for Red, etc.).
        #   4. Call `self.move_block(initial_pose, final_pose)` for each block
        #      in the sequence you design (e.g., color-by-color, or stacking).
        #
        # You are free to experiment with different strategies (single stack,
        # grouped stacks, or multiple stacks) as long as every call to
        # `move_block` follows a safe pick-and-place sequence.

        
        ################## YOUR CODE ENDS HERE ##################################

def main():

    """
    Entry point: set up ROS2, create nodes, and start the pipeline.

    - Initializes the ROS2 client library.
    - Creates a `UR3eController` node and an `ArucoTracker` node, then registers
      them in the global `NODES` list so they can be spun via `spin_all`.
    - Waits briefly for the controller to receive its first joint state.
    - Defines recommended drop-off poses (`dest_pose`) for Yellow, Red,
      and Blue blocks and passes them into a `BlockMover` instance.
    - Starts the block-processing logic by calling `block_mover.process_blocks`.
    """
    rclpy.init()

    try:

        # # Create an instance of UR3eController
        ur3e_controller = UR3eController()

        aruco_tracker = ArucoTracker(
            camera_matrix_path='/home/enme480_docker/enme480_ws/src/enme480_project/enme480_project/config/logitech_webcam_640x480.yaml',
            perspective_matrix_path='/home/enme480_docker/enme480_ws/src/enme480_project/enme480_project/perspective_matrix.npy'
        )

        # Register nodes for spin_all()
        global NODES
        NODES = [ur3e_controller, aruco_tracker]

        # wait for first joint state so controller is actually up
        start = time.time()
        while (not ur3e_controller.current_position_set) and rclpy.ok():
            if time.time() - start > 5.0:
                ur3e_controller.get_logger().warn(
                    "No joint state received within 5s. Continuing anyway."
                )
                break
            spin_all(0.1)

        # Define destination positions for each block. These are recommended sorting positions for different colors. The given z is the lowest level in the stack
        dest_pose = [
            (0.1, -0.1, 0.057, 0), # Yellow Drop Off 
            (0.2, -0.1, 0.057, 0), # Red Drop Off
            (0.3, -0.1, 0.057, 0), # Blue Drop Off
        ]

        # Initialize BlockMover and start moving blocks
        block_mover = BlockMover(ur3e_controller, aruco_tracker, dest_pose)
        block_mover.process_blocks(dest_pose)


    except KeyboardInterrupt:

        ur3e_controller.get_logger().info("Test interrupted by user.")

    finally:

        # Clean up and shut down
        ur3e_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
