#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Bool
from cv_bridge import CvBridge
from obstacle_challenge import ros_robot_controller_sdk as rrc
from ros_robot_controller_msgs.msg import ButtonState
import cv2
import numpy as np
import time

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.bridge = CvBridge()
        self.board = rrc.Board()

        # Configuration
        self.kp = 0.012
        self.kd = 0.002
        self.gain = 0.7
        self.straight_pwm = 1500
        self.throttle_pwm = 1615
        self.throttle_slow = 1610
        self.angle_pwm = 1500
        self.turn_dev = 30
        self.max_left = 1620
        self.max_right = 1380
        self.exit_threshold = 9500
        self.turn_threshold = 7000
        self.pillar_detection_thresh = 400
        self.pillar_clear_thresh = 100 
        self.pillar_timeout = 5.0      
        self.current_angle = 0.0
        self.turn_cooldown = 2.0
        self.stop_timer = 3.0

        self.left_roi = (0, 220, 180, 150)  # x, y, w, h
        self.right_roi = (460, 220, 180, 150)
        self.center_roi = (200, 200, 240, 200)

        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([179, 255, 255])
        self.lower_green = np.array([50, 150, 80])
        self.upper_green = np.array([95, 255, 255])
        self.lower_magenta = np.array([140, 100, 100])
        self.upper_magenta = np.array([170, 255, 255])
        
        # State
        self.last_turn_time = time.time()
        self.last_pillar_time = 0
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
        self.frame_ready = False
        self.pillar_state = None 
        self.pillar_mode = None  #COLOURS
        self.last_pillar_time = 0
        self.exit_start_time = 0
        self.following_error = 0  
        self.state = "navigate"
        

        """
        RGB COLOURS:
        GREEN: Green pillar detected
        RED: Red pillar detected
        BLUE: Turn detected
        MAGENTA: NOTHING DECTED PD steering

        DRIVE STATES:
        1. button: wait for button press
        2. start: escape parking lot
        3. navigate: navigate 3 laps around track
        4. park: parallel parking
        5. stop: shutdown all nodes and stop moving

        PILLAR STATES:
        1. Approach:
        2. Follow:
        3. Exit:
        """

        # ROS 2
        self.create_subscription(Image, '/image_raw', self.camera_callback, 10)
        self.create_subscription(Bool, '/lap_status', self.lap_callback, 10)
        self.create_subscription(Float32, '/imu_angle', self.imu_callback, 10)
        self.create_subscription(ButtonState, '/ros_robot_controller/button', self.button_callback, 10)
        self.stat_pub = self.create_publisher(String, '/state', 10)
        self.timer = self.create_timer(0.05, self.drive_callback)

        # Initialize board
        self.board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
        time.sleep(2)
        self.board.set_rgb([[1, 255, 0, 255], [2, 255, 0, 255]])  

    def button_callback(self, msg: ButtonState):
        if msg.id == 1 and msg.state == 1 and self.state== "button":
            self.state = "start"
            

    def lap_callback(self, msg):
        if msg.data and self.state != "park":
            self.get_logger().info("Received IMU signal: 3 laps complete, switching to parking mode.")
            self.state = "park"
            self.lap_done_time = time.time()
    
    def imu_callback(self, msg):
        self.current_angle = msg.data #use later

    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)

        
        left_crop = thresh[self.left_roi[1]:self.left_roi[1]+self.left_roi[3], 
                            self.left_roi[0]:self.left_roi[0]+self.left_roi[2]]
        right_crop = thresh[self.right_roi[1]:self.right_roi[1]+self.right_roi[3], 
                            self.right_roi[0]:self.right_roi[0]+self.right_roi[2]]
        
        left_color_crop = frame[self.left_roi[1]:self.left_roi[1]+self.left_roi[3], 
                                self.left_roi[0]:self.left_roi[0]+self.left_roi[2]]
        right_color_crop = frame[self.right_roi[1]:self.right_roi[1]+self.right_roi[3], 
                            self.right_roi[0]:self.right_roi[0]+self.right_roi[2]]
        center_color_crop = frame[self.center_roi[1]:self.center_roi[1]+self.center_roi[3], 
                            self.center_roi[0]:self.center_roi[0]+self.center_roi[2]]

        self.left_area = cv2.countNonZero(left_crop)
        self.right_area = cv2.countNonZero(right_crop)


        hsv_left = cv2.cvtColor(left_color_crop, cv2.COLOR_RGB2HSV)
        hsv_right = cv2.cvtColor(right_color_crop, cv2.COLOR_RGB2HSV)
        hsv_center = cv2.cvtColor(center_color_crop, cv2.COLOR_RGB2HSV)

        left_red_mask = cv2.bitwise_or(cv2.inRange(hsv_left, self.lower_red1, self.upper_red1), 
                                        cv2.inRange(hsv_left, self.lower_red2, self.upper_red2))
        center_red_mask = cv2.bitwise_or(cv2.inRange(hsv_center, self.lower_red1, self.upper_red1), 
                                        cv2.inRange(hsv_center, self.lower_red2, self.upper_red2))
        
        # Remove magenta from parking lot
        left_magenta_mask = cv2.inRange(hsv_left, self.lower_magenta, self.upper_magenta)
        center_magenta_mask = cv2.inRange(hsv_center, self.lower_magenta, self.upper_magenta)
        
        left_red_mask = cv2.subtract(left_red_mask, left_magenta_mask)
        center_red_mask = cv2.subtract(center_red_mask, center_magenta_mask)

        right_green_mask = cv2.inRange(hsv_right, self.lower_green, self.upper_green)
        center_green_mask = cv2.inRange(hsv_center, self.lower_green, self.upper_green)

        # Count pixels
        self.left_red_area = cv2.countNonZero(left_red_mask)
        self.right_green_area = cv2.countNonZero(right_green_mask)
        self.center_red_area = cv2.countNonZero(center_red_mask)
        self.center_green_area = cv2.countNonZero(center_green_mask)


        # DEBUG
        self.get_logger().info(f"Left ROI -> Black: {self.left_area}, Red: {self.left_red_area}")
        self.get_logger().info(f"Right ROI -> Black: {self.right_area}, Green: {self.right_green_area}")
        self.get_logger().info(f"Center ROI -> Red: {self.center_red_area}, Green: {self.center_green_area}")
        self.get_logger().info("-" * 40)

        # Pillar detection
        self.pillar_cx = None # reset every frame
        self.pillar_cy = None
        current_time = time.time()

        self.pillar_cx = None
        self.pillar_cy = None
        M_red = cv2.moments(center_red_mask)
        M_green = cv2.moments(center_green_mask)

        if self.pillar_mode is None:
            if (self.center_red_area > self.pillar_detection_thresh or 
                self.left_red_area > self.pillar_detection_thresh):
                self.pillar_mode = "red"
                self.pillar_state = "approach" 
                self.last_pillar_time = current_time
                if M_red['m00'] > 0:
                    self.pillar_cx = int(M_red['m10'] / M_red['m00'])
                    self.pillar_cy = int(M_red['m01'] / M_red['m00'])
                self.get_logger().info(f"RED pillar detected. APPROACH state. X: {self.pillar_cx}")
                self.board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
                
            elif (self.center_green_area > self.pillar_detection_thresh or 
                self.right_green_area > self.pillar_detection_thresh):
                self.pillar_mode = "green"  
                self.pillar_state = "approach"
                self.last_pillar_time = current_time
                if M_green['m00'] > 0:
                    self.pillar_cx = int(M_green['m10'] / M_green['m00'])
                    self.pillar_cy = int(M_green['m01'] / M_green['m00'])
                self.get_logger().info(f"GREEN pillar detected. APPROACH state. X: {self.pillar_cx}")
                self.board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])

        else:
            if self.pillar_mode == "red" and M_red['m00'] > 0:
                self.pillar_cx = int(M_red['m10'] / M_red['m00'])
                self.pillar_cy = int(M_red['m01'] / M_red['m00'])
                self.last_pillar_time = current_time
                current_pillar_area = self.center_red_area
            elif self.pillar_mode == "green" and M_green['m00'] > 0:
                self.pillar_cx = int(M_green['m10'] / M_green['m00'])
                self.pillar_cy = int(M_green['m01'] / M_green['m00'])
                self.last_pillar_time = current_time
                current_pillar_area = self.center_green_area
            else:
                if self.pillar_mode == "red":
                    current_pillar_area = max(self.center_red_area, self.left_red_area)
                else:
                    current_pillar_area = max(self.center_green_area, self.right_green_area)

            if current_pillar_area < self.pillar_clear_thresh:
                time_since_seen = current_time - self.last_pillar_time
                if time_since_seen > 0.5:
                    if self.pillar_state == "follow":
                        self.pillar_state = "exit"
                        self.exit_start_time = current_time
                        self.get_logger().info("Passed pillar. EXIT state.")
                    elif self.pillar_state == "approach":
                        self.get_logger().info("Lost pillar in APPROACH. Aborting.")
                        self.pillar_mode = None
                        self.pillar_state = None
                        self.board.set_rgb([[1, 255, 0, 255], [2, 255, 0, 255]])
            
            if self.pillar_state == "approach" and self.pillar_cx is not None:
                if (self.pillar_mode == "red" and self.pillar_cx < 120) or \
                   (self.pillar_mode == "green" and self.pillar_cx > 400):
                    self.pillar_state = "follow"
                    self.get_logger().info(f"Pillar FOLLOW state. X: {self.pillar_cx}")

            if self.pillar_state == "exit" and (current_time - self.exit_start_time) > 1.5:
                self.get_logger().info("Pillar maneuver complete.")
                self.pillar_mode = None
                self.pillar_state = None
                self.board.set_rgb([[1, 255, 0, 255], [2, 255, 0, 255]])

        self.frame_ready = True


        # Debug rois on frame
        cv2.rectangle(frame, (self.left_roi[0], self.left_roi[1]), 
                        (self.left_roi[0] + self.left_roi[2], self.left_roi[1] + self.left_roi[3]), (255, 0, 0), 2)
        cv2.rectangle(frame, (self.right_roi[0], self.right_roi[1]), 
                        (self.right_roi[0] + self.right_roi[2], self.right_roi[1] + self.right_roi[3]), (0, 255, 0), 2)
        cv2.rectangle(frame, (self.center_roi[0], self.center_roi[1]), 
                        (self.center_roi[0] + self.center_roi[2], self.center_roi[1] + self.center_roi[3]), (0, 0, 255), 2)

        cv2.imshow("Navigator Live Feed", frame)
        cv2.waitKey(1)

            
    def drive_callback(self):
        if not self.frame_ready:
            return
        
        if self.state == "start":
            self.state = "navigate"
            return
        
        # default speed
        self.throttle = self.throttle_turn if (self.left_turn or self.right_turn) else self.throttle_pwm
        # Default steering
        area_diff = self.right_area - self.left_area
        self.angle_pwm = int(self.straight_pwm + area_diff * self.kp + (area_diff - self.prev_diff) * self.kd)
        self.angle_pwm = max(min(self.angle_pwm, self.max_left), self.max_right)

        # Pillar Handling State Machine
        if self.pillar_mode is not None and self.pillar_state is not None:
            if self.pillar_mode == "red":
                if self.pillar_state == "approach":
                    if self.pillar_cx is not None:
                        error = self.pillar_cx - 100 
                        steering_adjustment = error * self.gain
                        self.angle_pwm = self.straight_pwm - steering_adjustment
                    throttle = self.throttle_slow
                    self.get_logger().info(f"RED APPROACH: CX={self.pillar_cx}, Steering={self.angle_pwm}")

                elif self.pillar_state == "follow":
                    self.angle_pwm = self.straight_pwm - 40
                    if self.center_red_area > 2500: 
                        self.angle_pwm -= 10 
                    elif self.center_red_area < 1500: 
                        self.angle_pwm += 10 
                    self.throttle = self.throttle_slow
                    self.get_logger().info(f"RED FOLLOW: Area={self.center_red_area}, Steering={self.angle_pwm}")

                elif self.pillar_state == "exit":
                    self.get_logger().info("RED EXIT: Returning to line following")

            elif self.pillar_mode == "green":
                if self.pillar_state == "approach":
                    if self.pillar_cx is not None:
                        error = self.pillar_cx - 400
                        steering_adjustment = error * self.gain
                        self.angle_pwm = self.straight_pwm - steering_adjustment
                    self.throttle = self.throttle_slow
                    self.get_logger().info(f"GREEN APPROACH: CX={self.pillar_cx}, Steering={self.angle_pwm}")

                elif self.pillar_state == "follow":
                    self.angle_pwm = self.straight_pwm + 40
                    self.throttle = self.throttle_slow
                    self.get_logger().info(f"GREEN FOLLOW: Area={self.center_green_area}, Steering={self.angle_pwm}")

                elif self.pillar_state == "exit":
                    self.get_logger().info("GREEN EXIT: Returning to line following")

        # turn handling
        elif not self.pillar_mode:  #only do turns if we're not in a pillar mode
            turn_diff = abs(self.left_area - self.right_area)
            if turn_diff >= self.turn_threshold:
                if self.right_area > self.left_area and not self.right_turn:
                    self.left_turn = True
                    self.board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
                    self.get_logger().info("Turning left")
                elif self.left_area > self.right_area and not self.left_turn:
                    self.right_turn = True
                    self.board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
                    self.get_logger().info("Turning right")

            if self.left_turn or self.right_turn:
                if (self.right_area >= self.exit_threshold and self.right_turn) or \
                   (self.left_area >= self.exit_threshold and self.left_turn):
                    if time.time() - self.last_turn_time >= self.turn_cooldown:
                        self.left_turn = self.right_turn = False
                        self.board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
                        self.last_turn_time = time.time()
                        self.prev_diff = 0

                elif self.left_turn:
                    self.angle_pwm = min(max(self.angle_pwm, self.straight_pwm + self.turn_dev), self.max_left)
                elif self.right_turn:
                    self.angle_pwm = max(min(self.angle_pwm, self.straight_pwm - self.turn_dev), self.max_right)

        state_msg = String()
        state_msg.data = self.state
        self.stat_pub(state_msg)

        if self.state == "park":
            if time.time() - self.lap_done_time >= self.stop_timer:
                self.get_logger().info("Preparing for parallel parking")
                self.speed(1500)
                self.steer_pwm(1500)
                return


        # Apply controls
        self.speed(self.throttle)
        self.steer_pwm(self.angle_pwm)

        # save states
        self.prev_diff = area_diff
        self.prev_angle = self.angle_pwm


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
        #Cleanup
        node.board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
        node.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
