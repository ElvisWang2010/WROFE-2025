import cv2
import numpy as np
import time
from picamera2 import Picamera2
import ros_robot_controller_sdk as rrc
import sys
import os
import subprocess
import threading

def listen_to_button_events():
    command = 'source /home/ubuntu/.zshrc && ros2 topic echo /ros_robot_controller/button'
    process = subprocess.Popen(
        ['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    while True:
        output = process.stdout.readline()
        if output:
            line = output.strip()
            if line.startswith("id:"):
                button_id = int(line.split(":")[1].strip())
            elif line.startswith("state:"):
                state = int(line.split(":")[1].strip())
                if state == 1 and button_id == 2:
                    print(f"Button {button_id} pressed")
                    return button_id
        time.sleep(0.1)
        
def check_node_status():
    command = 'source /home/ubuntu/.zshrc && ros2 topic list'
    result = subprocess.run(['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command], capture_output=True, text=True)
    return '/ros_robot_controller/button' in result.stdout

def wait_for_button_press():
    """Wait for button press using Docker ROS2 or fallback"""
    print("Waiting for button press to start...")
    
    # Check if ROS2 node is available
    while True:
        if check_node_status():
            print("ROS2 node detected")
            board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
            button_id = listen_to_button_events()
            print(f"Button {button_id} pressed! Starting challenge...")
            return True


if __name__ == '__main__':
    time.sleep(1)
    # === CONFIGURATION ===
    STRAIGHT_PWM = 1500
    THROTTLE_PWM = 1610
    KP = 0.08
    KD = 0.003
    MAX_LEFT = 1700
    MAX_RIGHT = 1300
    board = rrc.Board()
    # === STATE VARIABLES ===
    last_turn_time = time.time()
    prev_diff = 0
    prev_angle = STRAIGHT_PWM
    last_orange_time = 0
    orange_cooldown = 3.0
    stop_time = 0
    turns = 0
    lap_complete = False
  
    # === Initialize camera ===
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
    time.sleep(1)
    
    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([20, 255, 255])

    # ---- Define ROIs ----
    left_roi = (0, 200, 180, 170) # x, y, w, l
    right_roi = (460, 200, 180, 170)
    orange_roi = (100, 300, 440, 80) # x, y, w, h
    

    board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
    time.sleep(4)
    print("Board initialized")
   
    wait_for_button_press()
    board.set_rgb([[1, 255, 255, 0], [2, 255, 255, 0]]) 
    print("Starting")
    time.sleep(1)
    board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])

    # === Main loop ===
    while not lap_complete:
        # ---- Get camera frame ----
        frame = picam2.capture_array()
        x, y, w, h = orange_roi
        roi_crop = frame[y:y+h, x:x+w]

        # Convert to HSV for color detection
        hsv = cv2.cvtColor(roi_crop, cv2.COLOR_BGR2HSV)
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # Count orange pixels
        orange_pixel_count = cv2.countNonZero(mask_orange)

        if orange_pixel_count >= 400:  
            current_time = time.time()
            if current_time - last_orange_time > orange_cooldown:
                turns += 1
                last_orange_time = current_time
                print(f"Detected orange line - Turn count: {turns}")
                
        # Thresholding
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)

        mask = np.zeros_like(thresh)
        cv2.rectangle(mask, (left_roi[0], left_roi[1]), (left_roi[0]+left_roi[2], left_roi[1]+left_roi[3]), 255, -1)
        cv2.rectangle(mask, (right_roi[0], right_roi[1]), (right_roi[0]+right_roi[2], right_roi[1]+right_roi[3]), 255, -1)
        cv2.rectangle(mask, (orange_roi[0], orange_roi[1]), (orange_roi[0]+orange_roi[2], orange_roi[1]+orange_roi[3]), 255, 1)
        masked = cv2.bitwise_and(thresh, mask)

        contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Crop ROIs
        left_crop = thresh[left_roi[1]:left_roi[1]+left_roi[3], left_roi[0]:left_roi[0]+left_roi[2]]
        right_crop = thresh[right_roi[1]:right_roi[1]+right_roi[3], right_roi[0]:right_roi[0]+right_roi[2]]

        # Count black pixels
        left_area = cv2.countNonZero(left_crop)
        right_area = cv2.countNonZero(right_crop)

        # ---- PD steering ----
        area_diff = right_area - left_area
        angle_pwm = int(STRAIGHT_PWM + area_diff * KP + (area_diff - prev_diff) * KD)

        # ---- Drive ----
        angle_pwm = max(min(angle_pwm, MAX_LEFT), MAX_RIGHT)
        board.pwm_servo_set_position(0.1, [[4, angle_pwm], [2, THROTTLE_PWM]])        

        prev_diff = area_diff
        prev_angle = angle_pwm

        # ---- Camera only Stop ----
        if turns == 12: 
            print("Completed 3 laps. Stopping.")
            stop_time = time.time()
            turns += 1
        if turns >= 13:
            current_time = time.time()
            if current_time - stop_time >= 5:
                board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
                lap_complete = True


        # ---- debug view ----   
        if len(sys.argv) > 1 and sys.argv[1] == "debug":   
            print(f"Throttle: {THROTTLE_PWM}, Steering angle: {angle_pwm}")   
            cv2.rectangle(frame, (left_roi[0], left_roi[1]), (left_roi[0]+left_roi[2], left_roi[1]+left_roi[3]), (255, 0, 0), 2)
            cv2.rectangle(frame, (right_roi[0], right_roi[1]), (right_roi[0]+right_roi[2], right_roi[1]+right_roi[3]), (0, 0, 255), 2)
            cv2.rectangle(frame, (orange_roi[0], orange_roi[1]), (orange_roi[0]+orange_roi[2], orange_roi[1]+orange_roi[3]), (0, 165, 255), 2) 
            cv2.imshow("Live View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
                print("Pressed 'q' stopping program")
                break

    cv2.destroyAllWindows()

