import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import ros_robot_controller_sdk as rrc
import cv2
import numpy as np
import time


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.bridge = CvBridge()
        self.board = rrc.Board()

        # Configuration
        self.kp = 0.01
        self.kd = 0.002
        self.straight_pwm = 1500
        self.throttle_pwm = 1660
        self.throttle_turn = 1640
        self.turn_dev = 30
        self.max_left = 1620
        self.max_right = 1380
        self.exit_threshold = 9500
        self.turn_threshold = 2500
        self.turn_cooldown = 2.0
        self.stop_timer = 3.0

        # === ROIs (x, y, w, h) ===
        self.left_roi = (0, 200, 160, 180)
        self.right_roi = (510, 200, 160, 180)

        # State
        self.last_turn_time = time.time()
        self.prev_diff = 0
        self.prev_angle = self.straight_pwm
        self.left_area = 0
        self.right_area = 0
        self.laps = 0
        self.lap_done_time = None
        self.left_turn = False
        self.right_turn = False

        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.timer = self.create_timer(0.05, self.drive_callback)  # Runs every 0.05s

    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)

        # ROIs
        left_crop = thresh[self.left_roi[1]:self.left_roi[1]+self.left_roi[3],
                           self.left_roi[0]:self.left_roi[0]+self.left_roi[2]]
        right_crop = thresh[self.right_roi[1]:self.right_roi[1]+self.right_roi[3],
                            self.right_roi[0]:self.right_roi[0]+self.right_roi[2]]

        self.left_area = cv2.countNonZero(left_crop)
        self.right_area = cv2.countNonZero(right_crop)

    def drive_callback(self):
        if self.laps == 3:
            if time.time() - self.lap_done_time >= self.stop_timer:
                self.get_logger().info("preparing for parallel parking")
                self.speed(1500)
                self.steer_pwm(1500)
                return


def main(args=None):
    rclpy.init(args=args)
    navigator = NavigatorNode()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
