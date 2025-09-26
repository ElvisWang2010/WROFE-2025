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
        self.kp = 0.08
        self.kd = 0.003

        # Pillar 
        self.kp_pillar = 0.08
        self.kd_pillar = 0.003
        self.previous_pillar_error = 0
        self.pillar_cx = None
        self.pillar_cy = None
        self.pillar_detection_thresh = 700
        self.pillar_clear_thresh = 100 
        self.front_wall_threshold = 3000   
        self.side_wall_threshold = 4000
        self.screen_center_x = 320


        #Control
        self.straight_pwm = 1500
        self.throttle_pwm = 1610
        self.throttle_slow = 1605
        self.angle_pwm = 1500
        self.max_left = 1700
        self.max_right = 1300
        self.current_angle = 0.0
        #Park / Stop
        self.stop_timer = 3.0
        self.escape_timeout = 2  
        self.backup_timer = 3
        self.backup_time = 0

        self.left_roi = (0, 220, 180, 200)  # x, y, w, h
        self.right_roi = (460, 220, 180, 200)
        self.center_roi = (40, 80, 560, 320)  # x, y, w, h
        self.front_wall_roi = (300, 200, 140, 40)    # Upper center for front walls
        self.left_wall_roi = (80, 250, 120, 150)     # Left side walls
        self.right_wall_roi = (440, 250, 120, 150)   # Right side walls
        
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([179, 255, 255])
        self.lower_green = np.array([50, 150, 80])
        self.upper_green = np.array([95, 255, 255])
        self.lower_magenta = np.array([140, 100, 100])
        self.upper_magenta = np.array([170, 255, 255])
        
        """
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([5, 255, 255])
        self.lower_red2 = np.array([175, 130, 130])
        self.upper_red2 = np.array([179, 255, 255])
        self.lower_green = np.array([50, 150, 80])
        self.upper_green = np.array([95, 255, 255])
        self.lower_magenta = np.array([160, 50, 50])
        self.upper_magenta = np.array([175, 255, 255])
        """
    
        # State
        self.backup = False
        self.parking_side = None
        self.pillar_mode = None
        self.escape_phase = None
        self.escape_start_time = 0
        self.escape_phase_start = 0
        self.last_pillar_time = 0
        self.prev_diff = 0
        self.escape_attempts = 0
        self.prev_angle = self.straight_pwm
        self.left_area = 0
        self.right_area = 0
        self.left_red_area = 0
        self.center_red_area = 0
        self.right_green_area = 0
        self.center_green_area = 0
        self.lap_done_time = None
        self.frame_ready = False
        self.parking_side = None
        self.escape_mode = None
        self.wall_in_front = False
        self.mode = "button"

        """
        RGB COLOURS:
        GREEN: Green pillar detected
        RED: Red pillar detected
        MAGENTA: NOTHING DECTED PD steering
        STATES:
        1. button: wait for button press
        2. start: escape parking lot
        3. navigate: navigate 3 laps around track
        4. scan: look for parking lot
        4. park: parallel parking
        5. stop: shutdown all nodes and stop moving
        """
    
        # ROS 2
        self.create_subscription(Image, '/image_raw', self.camera_callback, 10)
        self.create_subscription(Bool, '/lap_status', self.lap_callback, 10)
        self.create_subscription(Float32, '/imu_angle', self.imu_callback, 10)
        self.create_subscription(ButtonState, '/ros_robot_controller/button', self.button_callback, 10)
        self.state_pub = self.create_publisher(String, '/state', 10)
        self.timer = self.create_timer(0.05, self.drive_callback)
        
        # Initialize board
        self.board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
        time.sleep(2)
        self.board.set_rgb([[1, 255, 0, 255], [2, 255, 0, 255]])
        
    def button_callback(self, msg: ButtonState):
        if msg.id == 2 and msg.state == 1 and self.mode == "button":
            self.mode = "navigate"
        else:
            self.board.set_rgb([[1, 0, 255, 255], [2, 0, 255, 255]])
            
            
    def lap_callback(self, msg):
        if msg.data and self.mode != 'park' and self.mode != 'scan':
            self.get_logger().info("Received IMU signal: 3 laps complete, switching to parking mode.")
            self.mode = 'scan'
            self.lap_done_time = time.time()
        
    def imu_callback(self, msg):
        self.current_angle = msg.data

    def camera_callback(self, msg):
        current_time = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)

        
        left_crop = thresh[self.left_roi[1]:self.left_roi[1]+self.left_roi[3], 
                            self.left_roi[0]:self.left_roi[0]+self.left_roi[2]]
        right_crop = thresh[self.right_roi[1]:self.right_roi[1]+self.right_roi[3], 
                            self.right_roi[0]:self.right_roi[0]+self.right_roi[2]]
        front_wall_crop = thresh[self.front_wall_roi[1]:self.front_wall_roi[1]+self.front_wall_roi[3], 
                           self.front_wall_roi[0]:self.front_wall_roi[0]+self.front_wall_roi[2]]
        left_wall_crop = thresh[self.left_wall_roi[1]:self.left_wall_roi[1]+self.left_wall_roi[3], 
                            self.left_wall_roi[0]:self.left_wall_roi[0]+self.left_wall_roi[2]]
        right_wall_crop = thresh[self.right_wall_roi[1]:self.right_wall_roi[1]+self.right_wall_roi[3], 
                            self.right_wall_roi[0]:self.right_wall_roi[0]+self.right_wall_roi[2]]
    
        
        center_color_crop = frame[self.center_roi[1]:self.center_roi[1]+self.center_roi[3], 
                            self.center_roi[0]:self.center_roi[0]+self.center_roi[2]]
        #Pd steering
        self.left_area = cv2.countNonZero(left_crop)
        self.right_area = cv2.countNonZero(right_crop)
        #pillar steering
        self.front_wall_area = cv2.countNonZero(front_wall_crop)
        self.left_wall_area = cv2.countNonZero(left_wall_crop)
        self.right_wall_area = cv2.countNonZero(right_wall_crop)

        hsv_center = cv2.cvtColor(center_color_crop, cv2.COLOR_RGB2HSV)

        
        center_magenta_mask = cv2.inRange(hsv_center, self.lower_magenta, self.upper_magenta)
        center_green_mask = cv2.inRange(hsv_center, self.lower_green, self.upper_green)
        center_red_mask = cv2.bitwise_or(cv2.inRange(hsv_center, self.lower_red1, self.upper_red1), 
                                cv2.inRange(hsv_center, self.lower_red2, self.upper_red2))
        
        # Count pixels
        self.center_magenta_area = cv2.countNonZero(center_magenta_mask)
        self.center_green_area = cv2.countNonZero(center_green_mask)
        self.center_red_area = cv2.countNonZero(center_red_mask)
    
        if self.front_wall_area >= self.front_wall_threshold:
            self.wall_in_front = True
            #self.get_logger().info("Wall is close")
        else:
            self.wall_in_front = False
            #self.get_logger().info("Wall is no longer close")
            
        # Pillar detection
        current_time = time.time()
        if self.center_red_area > self.pillar_detection_thresh and self.center_red_area > self.center_green_area:
            self.pillar_mode = "red"
            self.last_pillar_time = current_time
            self.prev_pillar_error = self.center_red_area
            self.get_logger().info("Entering RED pillar mode")
            self.board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
            
        elif self.center_green_area > self.pillar_detection_thresh and self.center_green_area > self.center_red_area:
            self.pillar_mode = "green"  
            self.last_pillar_time = current_time
            self.prev_pillar_error = self.center_green_area
            self.get_logger().info("Entering GREEN pillar mode")
            self.board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])

        else:
            #Exit condition
            current_pillar_area = self.center_red_area if self.pillar_mode == "red" else self.center_green_area
            # Use hysteresis to prevent flickering
            if (current_pillar_area < self.pillar_clear_thresh and 
                current_time - self.last_pillar_time > 0.5 and self.pillar_mode is not None):  # Wait 0.5s of low signal before confirming pillar gone
                self.get_logger().info(f"Exiting {self.pillar_mode} pillar mode")
                self.pillar_mode = None
                self.board.set_rgb([[1, 255, 0, 255], [2, 255, 0, 255]])


        self.pillar_cx = None  # Reset centroid each frame
        self.pillar_cy = None

        if self.pillar_mode == "red":
            # Find centroid
            M = cv2.moments(center_red_mask)
            if M["m00"] > 0:
                self.pillar_cx = int(M["m10"] / M["m00"])
                self.pillar_cy = int(M["m01"] / M["m00"])

        elif self.pillar_mode == "green":
            # Find centroid
            M = cv2.moments(center_green_mask)
            if M["m00"] > 0:
                self.pillar_cx = int(M["m10"] / M["m00"])
                self.pillar_cy = int(M["m01"] / M["m00"])

        if self.mode == "scan" and self.center_magenta_area > 5000:
            self.mode = "park"

        if self.mode == "start":
            if self.left_area > self.right_area:
                self.parking_side = "left"
                self.get_logger().info("Parking lot detected on LEFT")
            elif self.right_area > self.left_area:
                self.parking_side = "right" 
                self.get_logger().info("Parking lot detected on RIGHT")
            self.escape_phase = "front"


        # DEBUG
        if self.pillar_mode is not None:
            self.get_logger().info(f"CENTROID X: {self.pillar_cx}, CENTROID Y: {self.pillar_cy}")
        self.get_logger().info(f"Center ROI -> Red: {self.center_red_area}, Green: {self.center_green_area}")
        self.get_logger().info("-" * 40)
    
        # Debug rois on frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.rectangle(frame, (self.left_roi[0], self.left_roi[1]), 
                        (self.left_roi[0] + self.left_roi[2], self.left_roi[1] + self.left_roi[3]), (255, 0, 0), 2)
        cv2.rectangle(frame, (self.right_roi[0], self.right_roi[1]), 
                        (self.right_roi[0] + self.right_roi[2], self.right_roi[1] + self.right_roi[3]), (0, 255, 0), 2)
        cv2.rectangle(frame, (self.center_roi[0], self.center_roi[1]), 
                        (self.center_roi[0] + self.center_roi[2], self.center_roi[1] + self.center_roi[3]), (0, 0, 255), 2)
        

        if self.pillar_cx is not None and self.pillar_cy is not None:
            cv2.circle(frame, (self.pillar_cx, self.pillar_cy), 10, (0, 255, 255), -1)  
            cv2.putText(frame, f"CX: {self.pillar_cx}", (self.pillar_cx + 15, self.pillar_cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.rectangle(frame, (self.front_wall_roi[0], self.front_wall_roi[1]), 
                        (self.front_wall_roi[0] + self.front_wall_roi[2], self.front_wall_roi[1] + self.front_wall_roi[3]), 
                        (255, 255, 0), 2)  
            cv2.rectangle(frame, (self.left_wall_roi[0], self.left_wall_roi[1]), 
                        (self.left_wall_roi[0] + self.left_wall_roi[2], self.left_wall_roi[1] + self.left_wall_roi[3]), 
                        (255, 0, 255), 2) 
            cv2.rectangle(frame, (self.right_wall_roi[0], self.right_wall_roi[1]), 
                        (self.right_wall_roi[0] + self.right_wall_roi[2], self.right_wall_roi[1] + self.right_wall_roi[3]), 
                        (0, 255, 255), 2)  

            cv2.putText(frame, f"Centroid X: {self.pillar_cx}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
   
        cv2.putText(frame, f"State: {self.mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"Pillar: {self.pillar_mode}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Navigator Live Feed", frame)
        cv2.waitKey(1)

        self.frame_ready = True
    
    def drive_callback(self):
        if not self.frame_ready or self.mode == "button":
            return
        
        current_time = time.time()
        if self.mode == "start":
            # Initialize escape timing
            if self.escape_start_time == 0:
                self.escape_start_time = current_time
                self.escape_phase_start = current_time
                self.escape_mode = "escape"
                self.get_logger().info("Escape maneuver started")
            
            # Calculate time elapsed in current phase
            phase_elapsed = current_time - self.escape_phase_start
            
            # STATE TRANSITIONS
            if phase_elapsed >= self.escape_timeout:
                if self.escape_mode == "escape":
                    self.escape_mode = "correct"
                    self.escape_phase_start = current_time
                    self.get_logger().info(f"Switching escape mode to: {self.escape_mode}")
                elif self.escape_mode == "correct":
                    self.escape_mode = "backup"
                    self.escape_phase_start = current_time
                    self.get_logger().info(f"Switching escape mode to: {self.escape_mode}")
                elif self.escape_mode == "correct":
                    self.escape_mode = "backup"
                    self.escape_phase_start = current_time
                    self.get_logger().info(f"Switching escape mode to: {self.escape_mode}")
                else:
                    self.mode = "navigate"
                    return
    
            if self.parking_side == "left":
                if self.escape_mode == "escape":
                    self.angle_pwm = self.max_right
                    self.throttle = 1570
                elif self.escape_mode == "correct":
                    self.angle_pwm = 1400
                    self.throttle = 1565
                else:
                    self.angle_pwm = 1500
                    self.throttle = 1450
            else:  # parking_side == "right"
                if self.escape_mode == "escape":
                    self.angle_pwm = self.max_left
                    self.throttle = 1570
                    self.get_logger().info(f"RIGHT escape forwards ({phase_elapsed:.1f}s)")
                elif self.escape_mode == "correct":
                    self.angle_pwm = 1600
                    self.throttle = 1565
                else:
                    self.angle_pwm = 1500
                    self.throttle = 1450

            # Apply controls and return
            self.speed(self.throttle)
            self.steer_pwm(self.angle_pwm)
            return

        if self.pillar_mode and self.mode != "start" and self.mode != "button" and self.mode != "stop":
            current_time = time.time()
            
            if self.pillar_cx is not None:  # Only steer if centroid is detected

                if self.pillar_mode == "red":
                    error = self.pillar_cx - (self.screen_center_x + 100) 
                    steering_adjustment = error * 2.0
                    self.angle_pwm = self.straight_pwm + steering_adjustment
                    if self.wall_in_front:
                        self.angle_pwm = self.max_left
                        print("emergency left turn")
                    self.get_logger().info(f"Red pillar - CX: {self.pillar_cx}, Error: {error}, PWM: {self.angle_pwm}")

                    
                elif self.pillar_mode == "green":
                    error = self.pillar_cx - (self.screen_center_x - 100)  
                    steering_adjustment = error * 2.0
                    self.angle_pwm = self.straight_pwm + steering_adjustment
                    if self.wall_in_front:
                        self.angle_pwm = self.max_right
                        print("emergency right turn")
                    
                    self.get_logger().info(f"Green pillar - CX: {self.pillar_cx}, Error: {error}, PWM: {self.angle_pwm}")
                
            else:
                # If centroid lost but still in pillar mode.
                if self.pillar_mode == "red":
                    self.angle_pwm = self.straight_pwm - 60  
                else:
                    self.angle_pwm = self.straight_pwm + 60  
                self.get_logger().info(f"{self.pillar_mode} pillar - Centroid lost, using default turn")
                error = self.prev_pillar_error
            # Apply limits
            self.angle_pwm = max(min(self.angle_pwm, self.max_left), self.max_right)
            self.throttle = self.throttle_slow
            self.prev_pillar_error = error
        else:
            self.board.set_rgb([[1, 255, 0, 255], [2, 255, 0, 255]])
            area_diff = self.right_area - self.left_area
            self.angle_pwm = int(self.straight_pwm + area_diff * self.kp + (area_diff - self.prev_diff) * self.kd)
            self.angle_pwm = max(min(self.angle_pwm, self.max_left), self.max_right)
            self.throttle = self.throttle_pwm
            self.prev_diff = area_diff
        

        # Park
        if self.mode == "park":
            if time.time() - self.lap_done_time >= self.stop_timer:
                self.get_logger().info("Preparing for parallel parking")
                self.speed(1500)
                self.steer_pwm(1500)
                msg_out = String()
                msg_out.data = "stop"
                self.state_pub.publish(msg_out) #stop for now
                rclpy.shutdown()
                return

        # Apply controls
        self.speed(self.throttle)
        self.steer_pwm(self.angle_pwm)

    def speed(self, throttle):
        self.board.pwm_servo_set_position(0.1, [[2, throttle]])

    def steer_pwm(self, angle):
        self.board.pwm_servo_set_position(0.1, [[4, int(angle)]])

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
        node.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
