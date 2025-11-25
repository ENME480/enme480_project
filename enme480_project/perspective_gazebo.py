#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerspectiveCalibrator(Node):
    def __init__(self):
        """
        ROS2 node to compute and save a perspective transform for the Gazebo camera.

        - Subscribes to the `/camera` topic and displays the incoming frames
          in an OpenCV window.
        - Lets the user click four points in the image that correspond to
          known points on the table in the world (table) frame.
        - Once four points are selected, computes a 3x3 homography matrix
          that maps image coordinates to table-frame coordinates and saves
          it to `self.output_path` for later use (e.g., by `ArucoTracker`).
        """
        super().__init__('perspective_calibrator')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/camera',    # ROS2 topic from Gazebo
            self.image_callback,
            10
        )

        self.selected_points = []
        self.perspective_matrix = None

        self.window_name = 'Camera Frame'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        # Same table-frame points as your original code
        self.points_in_table_frame = np.array(
            [[-253, -410.5], [553, -410.5], [553, 710.5], [-253, 710.5]],
            dtype=np.float32
        )

        # Same save path you used later
        self.output_path = (
            '/home/enme480_docker/enme480_ws/src/enme480_project_solved/'
            'enme480_project_solved/perspective_matrix.npy'
        )

    def mouse_callback(self, event, x, y, flags, param):
        """
        OpenCV mouse callback used to collect four image points.

        - On each left mouse button click, appends the (x, y) pixel location
          to `self.selected_points` until four points are stored.
        - Logs each selected point using the ROS2 logger so you can verify
          the order and location of your calibration clicks.
        - These four points are paired with `self.points_in_table_frame`
          to build the perspective transform in `image_callback`.
        """
        if event == cv2.EVENT_LBUTTONDOWN and len(self.selected_points) < 4:
            self.selected_points.append((x, y))
            self.get_logger().info(
                f'Selected point {len(self.selected_points)}: ({x}, {y})'
            )

    def image_callback(self, msg: Image):
        """
        ROS2 image subscription callback for building and applying the homography.

        - Converts the incoming ROS2 `Image` message to a BGR frame.
        - Draws any user-selected calibration points on the image.
        - Once exactly four points have been selected and no matrix is stored
          yet, computes the perspective transform between the image points and
          `self.points_in_table_frame`, then saves it to disk.
        - If a perspective matrix exists, also warps the image and displays
          the warped view with a reference point overlaid for debugging.
        - Handles keyboard input: pressing 'q' will log a message and shut
          down the ROS2 node cleanly.
        """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Draw selected points
        for p in self.selected_points:
            cv2.circle(frame, p, 3, (0, 0, 255), -1)

        # Once we have 4 points, compute the matrix once
        if len(self.selected_points) == 4 and self.perspective_matrix is None:
            pts_img = np.array(self.selected_points, dtype=np.float32)
            self.perspective_matrix = cv2.getPerspectiveTransform(
                pts_img,
                self.points_in_table_frame
            )
            np.save(self.output_path, self.perspective_matrix)
            self.get_logger().info(
                f'Perspective matrix saved to {self.output_path}'
            )

        # Show warped image if matrix is available
        if self.perspective_matrix is not None:
            result = cv2.warpPerspective(frame, self.perspective_matrix, (550, 450))
            cv2.circle(result, (240, 130), 3, (255, 0, 0), -1)
            cv2.imshow('frame1', result)

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Exiting.')
            rclpy.shutdown()

def main(args=None):
    """
    Standalone entry point to run the Gazebo perspective calibration node.

    - Initializes ROS2, creates a `PerspectiveCalibrator` node, and spins it
      so that incoming `/camera` images are processed and displayed.
    - Use this script to generate the `perspective_matrix.npy` file by
      clicking four correspondences between the image and the table frame
      before running the main pick-and-place pipeline.
    """
    rclpy.init(args=args)
    node = PerspectiveCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
