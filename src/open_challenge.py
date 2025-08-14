import cv2
import numpy as np
import time
from picamera2 import Picamera2
import ros_robot_controller_sdk as rrc


if __name__ == '__main__':
    time.sleep(1)
    # === CONFIGURATION ===
    STRAIGHT_PWM = 1500
    THROTTLE_PWM = 1640
    THROTTLE_TURN = 1640
    TURN_THRESHOLD = 3000
    EXIT_THRESHOLD = 9500
    KP = 0.012
    KD = 0.001
    MAX_LEFT = 1620
    MAX_RIGHT = 1380
    TURN_DEV = 30
    board = rrc.Board()
    # === STATE VARIABLES ===
    last_turn_time = time.time()
    prev_diff = 0
    prev_angle = STRAIGHT_PWM
    left_turn = False
    right_turn = False
    last_orange_time = 0
    orange_cooldown = 2
    stop_time = 0

    turns = 0
    lap_complete = False
    
    board.set_rgb([[1, 255, 0, 0]]) #RED
    print("Received start command")
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
    left_roi = (0, 200, 180, 180) # x, y, w, l
    right_roi = (460, 200, 180, 180)
    orange_roi = (100, 360, 440, 40)  # x, y, w, h
    
    board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
    time.sleep(5)
    board.set_rgb([[1, 255, 255, 0]]) #YELLOW
    print("Board initialized")    

    board.set_rgb([[1, 0, 0, 0]]) #OFF
    print("Starting")
    time.sleep(1)
    board.set_rgb([[1, 0, 255, 0]]) #GREEN
    board.set_rgb([[2, 0, 255, 0]]) #GREEN
    
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

        if orange_pixel_count >= 500:  
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

        # ---- Turn state logic ----
        if left_area <= TURN_THRESHOLD and not right_turn:
            left_turn = True
            board.set_rgb([[1, 0, 0, 255]])
            board.set_rgb([[2, 0, 0, 255]])
            print("Turning left")
        elif right_area <= TURN_THRESHOLD and not left_turn:
            right_turn = True
            board.set_rgb([[1, 0, 0, 255]]) 
            board.set_rgb([[2, 0, 0, 255]])
            print("Turning right")

        if left_turn or right_turn:
            if (right_area >= EXIT_THRESHOLD and right_turn) or (left_area >= EXIT_THRESHOLD and left_turn):
                current_time = time.time()
                if current_time - last_turn_time >= 1.4:
                    left_turn = right_turn = False
                    board.set_rgb([[1, 0, 255, 0]])
                    board.set_rgb([[2, 0, 255, 0]])
                    last_turn_time = current_time
                    prev_diff = 0
                    print(f"Turn complete. Segments = {turns}")

            elif left_turn:
                angle_pwm = min(max(angle_pwm, STRAIGHT_PWM + TURN_DEV), MAX_LEFT)
            elif right_turn:
                angle_pwm = max(min(angle_pwm, STRAIGHT_PWM - TURN_DEV), MAX_RIGHT)
        else:
            angle_pwm = max(min(angle_pwm, MAX_LEFT), MAX_RIGHT)

        # ---- Drive ----
        if left_turn or right_turn:
            board.pwm_servo_set_position(0.1, [[4, angle_pwm], [2, THROTTLE_TURN]])
        else:
            board.pwm_servo_set_position(0.1, [[4, angle_pwm], [2, THROTTLE_PWM]])
        print(f"Steering angle: {angle_pwm}")                                              
        prev_diff = area_diff
        prev_angle = angle_pwm



        # ---- Camera only Stop ----
        if turns == 12: 
            print("Completed 3 laps. Stopping.")
            stop_time = time.time()
            turns += 1
        if turns >= 13:
            current_time = time.time()
            if current_time - stop_time >= 2.5:
                board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
                lap_complete = True


        # ---- debug view ----      
        cv2.rectangle(frame, (left_roi[0], left_roi[1]), (left_roi[0]+left_roi[2], left_roi[1]+left_roi[3]), (255, 0, 0), 2)
        cv2.rectangle(frame, (right_roi[0], right_roi[1]), (right_roi[0]+right_roi[2], right_roi[1]+right_roi[3]), (0, 0, 255), 2)
        cv2.imshow("Live View", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
            print("Pressed 'q' stopping program")
            break

    cv2.destroyAllWindows()
    GPIO.cleanup()

