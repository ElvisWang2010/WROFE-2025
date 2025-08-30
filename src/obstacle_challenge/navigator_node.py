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

        # Configuration
        self.kp = 0.012
        self.kd = 0.002
        self.kp_pillar = 0.005
        self.kd_pillar = 0.001
        self.previous_pillar_error = 0
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
        self.pillar_mode = None
        self.mode = "start"
        self.parking_mode = False  

        """
        RGB COLOURS:
        GREEN: Green pillar detected
        RED: Red pillar detected
        BLUE: Turn detected
        MAGENTA: NOTHING DECTED PD steering
        """

        # ROS 2
        self.create_subscription(Image, '/image_raw', self.camera_callback, 10)
        self.create_subscription(Bool, '/imu_status', self.lap_callback, 10)
        self.create_subscription(Float32, '/imu_angle', self.imu_callback, 10)
        self.stop_pub = self.create_publisher(Bool, '/stop_signal', 10)
        self.timer = self.create_timer(0.05, self.drive_callback)

        # Initialize board
        self.board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
        time.sleep(2)
        self.board.set_rgb([[1, 255, 0, 255], [2, 255, 0, 255]])  


    def lap_callback(self, msg):
        if msg.data and not self.parking_mode:
            self.get_logger().info("Received IMU signal: 3 laps complete, switching to parking mode.")
            self.mode = 'park'
            self.parking_mode = True
            self.lap_done_time = time.time()
    
    def imu_callback(self, msg):
        self.current_angle = msg.data

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
        current_time = time.time()
        if self.pillar_mode is None:
            if (self.center_red_area > self.pillar_detection_thresh or 
                self.left_red_area > self.pillar_detection_thresh):
                self.pillar_mode = "red"
                self.last_pillar_time = current_time
                self.prev_pillar_error = max(self.center_red_area, self.left_red_area)
                self.get_logger().info("Entering RED pillar mode")
                self.board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
                
            elif (self.center_green_area > self.pillar_detection_thresh or 
                self.right_green_area > self.pillar_detection_thresh):
                self.pillar_mode = "green"  
                self.last_pillar_time = current_time
                self.prev_pillar_error = max(self.center_green_area, self.right_green_area)
                self.get_logger().info("Entering GREEN pillar mode")
                self.board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])

        else:
            # use strongest signal
            if self.pillar_mode == "red":
                current_pillar_area = max(self.center_red_area, self.left_red_area)
            else:
                current_pillar_area = max(self.center_green_area, self.right_green_area)
            
            # Exit conditions
            if (current_pillar_area < self.pillar_clear_thresh or 
                current_time - self.last_pillar_time > self.pillar_timeout):
                self.get_logger().info(f"Exiting {self.pillar_mode} pillar mode")
                self.pillar_mode = None
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
        
        if self.mode == "start":
            self.mode = "navigate"
            return

        # PD Pillar steering
        if self.pillar_mode:
            if self.pillar_mode == "red":
                #red pillar on left
                if self.left_red_area > self.center_red_area:
                    error = self.left_red_area
                    base_angle = self.straight_pwm + 40  
                    steering_adjustment = error * self.kp_pillar * 0.8  t
                    self.get_logger().info("Red pillar on LEFT - gentle left turn")
                else:
                    #red pillar center
                    error = self.center_red_area
                    base_angle = self.straight_pwm - 60 
                    steering_adjustment = error * self.kp_pillar
                    self.get_logger().info("Red pillar in CENTER - right turn")
                self.angle_pwm = int(base_angle - steering_adjustment) 
                self.throttle = self.throttle_slow
                self.prev_pillar_error = error
                self.get_logger().info(f"Red: steering angle={self.angle_pwm}, throttle={self.throttle}")


            elif self.pillar_mode == "green":
                #green pillar on right
                if self.right_green_area > self.center_green_area:
                    error = self.right_green_area
                    base_angle = self.straight_pwm - 40 
                    steering_adjustment = error * self.kp_pillar * 0.8
                    self.get_logger().info("Green pillar on RIGHT - gentle right turn")
                else:
                    #green pillar center
                    error = self.center_green_area
                    base_angle = self.straight_pwm + 60 
                    steering_adjustment = error * self.kp_pillar
                    self.get_logger().info("Green pillar in CENTER - left turn")
                
                self.angle_pwm = int(base_angle + steering_adjustment)
                self.throttle = self.throttle_slow
                self.prev_pillar_error = error
                self.get_logger().info(f"Green: steering angle={self.angle_pwm}, throttle={self.throttle}")

            self.angle_pwm = max(min(self.angle_pwm, self.max_left), self.max_right)
    
        else: # Normal PD steering
            self.board.set_rgb([[1, 255, 0, 255], [2, 255, 0, 255]])
            area_diff = self.right_area - self.left_area
            self.angle_pwm = int(self.straight_pwm + area_diff * self.kp + (area_diff - self.prev_diff) * self.kd)
            self.angle_pwm = max(min(self.angle_pwm, self.max_left), self.max_right)
            self.throttle = self.throttle_pwm
            self.prev_diff = area_diff

        # Turn detection
        if not self.pillar_mode:
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

                elif self.left_turn:
                    self.angle_pwm = min(max(self.angle_pwm, self.straight_pwm + self.turn_dev), self.max_left)
                elif self.right_turn:
                    self.angle_pwm = max(min(self.angle_pwm, self.straight_pwm - self.turn_dev), self.max_right)

        # Park
        if self.mode == "park":
            if time.time() - self.lap_done_time >= self.stop_timer:
                self.get_logger().info("Preparing for parallel parking")
                self.speed(1500)
                self.steer_pwm(1500)
                msg_out = Bool()
                msg_out.data = True
                self.stop_pub.publish(msg_out) #stop for now
                return

        # Apply controls
        self.speed(self.throttle)
        self.steer_pwm(self.angle_pwm)


    #Helper Functions
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
        # cleanup
        node.board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
        node.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
