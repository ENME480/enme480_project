#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerspectiveCalibrator(Node):
    def __init__(self):
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
        if event == cv2.EVENT_LBUTTONDOWN and len(self.selected_points) < 4:
            self.selected_points.append((x, y))
            self.get_logger().info(
                f'Selected point {len(self.selected_points)}: ({x}, {y})'
            )

    def image_callback(self, msg: Image):
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
    rclpy.init(args=args)
    node = PerspectiveCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
