import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
from obstacle_challenge import ros_robot_controller_sdk as rrc
import cv2
import numpy as np
import time


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.bridge = CvBridge()
        self.board = rrc.Board()

        # === Configuration ===
        self.kp = 0.012
        self.kd = 0.002
        self.straight_pwm = 1500
        self.throttle_pwm = 1615
        self.throttle_turn = 1615
        self.turn_dev = 30
        self.max_left = 1620
        self.max_right = 1380
        self.exit_threshold = 9500
        self.turn_threshold = 5000
        self.pillar_detection_thresh = 700
        self.current_angle = 0.0
        self.turn_cooldown = 2.0
        self.stop_timer = 3.0

        # === ROIs (x, y, w, h) ===
        self.left_roi = (0, 240, 180, 150) # x, y, w, l
        self.right_roi = (460, 240, 180, 150)
        self.center_roi = (200, 150, 240, 120)

        self.lower_red1,self. upper_red1 = (0, 100, 100), (10, 255, 255)
        self.lower_red2, self.upper_red2 = (160, 100, 100), (179, 255, 255)
        self.lower_green, self.upper_green = (50, 150, 80), (95, 255, 255)

        # === State ===
        self.last_turn_time = time.time()
        self.prev_diff = 0
        self.prev_angle = self.straight_pwm
        self.left_area = 0
        self.right_area = 0
        self.left_red_area = 0
        self.center_red_area = 0
        self.right_green_area = 0
        self.center_green_area = 0
        self.lap_done_time = None
        self.left_turn = False
        self.right_turn = False
        self.pillar_mode = "none"
        self.mode = "navigate"


        # ROS 2
        self.create_subscription(Image, '/image_raw', self.camera_callback, 10)
        self.create_subscription(Bool, '/imu_status', self.lap_callback, 10)
        self.gyro_sub = self.create_subscription(Float32, '/imu_angle', self.imu_callback, 10)
        self.timer = self.create_timer(0.05, self.drive_callback)  # Runs every 0.05s



    def lap_callback(self, msg):
        if msg.data and not self.parking_mode:
            self.get_logger().info("Received IMU signal: 3 laps complete, switching to parking mode.")
            self.mode = 'park'
    
    def imu_callback(self, msg):
        self.current_angle = msg.data
            
    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)

        # ROIs
        left_crop = thresh[self.left_roi[1]:self.left_roi[1]+self.left_roi[3],
                           self.left_roi[0]:self.left_roi[0]+self.left_roi[2]]
        right_crop = thresh[self.right_roi[1]:self.right_roi[1]+self.right_roi[3],
                            self.right_roi[0]:self.right_roi[0]+self.right_roi[2]]
        center_crop = frame[self.center_roi[1]:self.center_roi[1]+self.center_roi[3],
                            self.center_roi[0]:self.center_roi[0]+self.center_roi[2]]

        self.left_area = cv2.countNonZero(left_crop)
        self.right_area = cv2.countNonZero(right_crop)


        hsv_left = cv2.cvtColor(frame[self.left_roi[1]:self.left_roi[1]+self.left_roi[3], self.left_roi[0]:self.left_roi[0]+self.left_roi[2]], cv2.COLOR_BGR2HSV)
        hsv_right = cv2.cvtColor(frame[self.right_roi[1]:self.right_roi[1]+self.right_roi[3], self.right_roi[0]:self.right_roi[0]+self.right_roi[2]], cv2.COLOR_BGR2HSV)
        hsv_center = cv2.cvtColor(center_crop, cv2.COLOR_BGR2HSV)


        left_red_mask = cv2.bitwise_or(cv2.inRange(hsv_left, self.lower_red1, self.upper_red1), cv2.inRange(hsv_left, self.lower_red2, self.upper_red2))
        center_red_mask = cv2.bitwise_or(cv2.inRange(hsv_center, self.lower_red1, self.upper_red1), cv2.inRange(hsv_center, self.lower_red2, self.upper_red2))
        right_green_mask = cv2.inRange(hsv_right, self.lower_green, self.upper_green)
        center_green_mask = cv2.inRange(hsv_center, self.lower_green, self.upper_green)

        left_red_area = cv2.countNonZero(left_red_mask)
        right_green_area = cv2.countNonZero(right_green_mask)
        center_red_area = cv2.countNonZero(center_red_mask)
        center_green_area = cv2.countNonZero(center_green_mask)
    
        if center_red_area > self.pillar_detection_thresh:
            self.pillar_mode = "red"
            self.get_logger().info("Entering RED pillar mode")
            self.board.set_rgb([[1, 255, 0, 255], [2, 255, 0, 0]])
        elif center_green_area > self.pillar_detection_thresh:
            self.pillar_mode = "green"
            self.get_logger().info("Entering GREEN pillar mode")
            self.board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])

    def drive_callback(self):
        # PD
        if self.pillar_mode == "none":
            area_diff = self.right_area - self.left_area
            angle_pwm = int(self.straight_pwm + area_diff * self.kp + (area_diff - self.prev_diff) * self.kd)

        # Pillar detection
        if self.pillar_mode is not None:
            if self.pillar_mode == "red":   
                if self.center_red_area >= self.pillar_detection_thresh:
                    angle_pwm = self.max_right
                elif self.left_red_area >= self.pillar_detection_thresh:
                    angle_pwm = self.max_left
            elif self.pillar_mode == "green":
                if self.center_green_area > self.pillar_detection_thresh:
                    angle_pwm = self.max_left
                elif self.right_green__area >= self.pillar_detection_thresh:
                    angle_pwm = self.max_right
        else:
            angle_pwm = max(min(angle_pwm, self.max_left), self.max_right)

        if self.mode == "park":
            if time.time() - self.lap_done_time >= self.stop_timer:
                self.get_logger().info("preparing for parallel parking")
                self.speed(1500)
                self.steer_pwm(1500)
                return
            
        # Turns
        if self.pillar_mode == "none":
            if self.left_area <= self.turn_threshold and not self.right_turn:
                self.left_turn = True
                self.board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
                self.get_logger().info("Turning left")
            elif self.right_area <= self.turn_threshold and not self.left_turn:
                self.right_turn = True
                self.board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
                self.get_logger().info("Turning right")

            if self.left_turn or self.right_turn:
                if (self.right_area >= self.exit_threshold and self.right_turn) or \
                (self.left_area >= self.exit_threshold and self.left_turn) or \
                    self.pillar_mode != "none": 
                    if time.time() - self.last_turn_time >= self.turn_cooldown:
                        self.left_turn = self.right_turn = False
                        self.board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
                        self.last_turn_time = time.time()
                        self.prev_diff = 0

                elif self.left_turn:
                    angle_pwm = min(max(angle_pwm, self.straight_pwm + self.turn_dev), self.max_left)
                elif self.right_turn:
                    angle_pwm = max(min(angle_pwm, self.straight_pwm - self.turn_dev), self.max_right)
        
        # Drive 
        throttle = self.throttle_turn if (self.left_turn or self.right_turn) else self.throttle_pwm
        self.speed(throttle)
        self.steer_pwm(angle_pwm)

        # Save state
        self.prev_diff = area_diff
        self.prev_angle = angle_pwm


    def speed(self, throttle):
        self.board.pwm_servo_set_position(0.1, [[2, throttle]])

    def steer_pwm(self, angle):
        self.board.pwm_servo_set_position(0.1, [[4, angle]])


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
