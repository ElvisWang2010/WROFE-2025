import cv2
import numpy as np
import time
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import ros_robot_controller_sdk as rrc
#import sys


if __name__ == '__main__':
    time.sleep(1)
    # === CONFIGURATION ===
    STRAIGHT_PWM = 1500
    THROTTLE_PWM = 1660
    THROTTLE_TURN = 1640
    TURN_THRESHOLD = 3000
    EXIT_THRESHOLD = 9000
    KP = 0.06
    KD = 0.003
    MAX_LEFT = 1650
    MAX_RIGHT = 1350
    TURN_DEV = 50
    board = rrc.Board()
    # === STATE VARIABLES ===
    prev_diff = 0
    prev_angle = STRAIGHT_PWM
    left_turn = False
    right_turn = False

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
    # ---- Define ROIs ----
    left_roi = (0, 200, 160, 180) # x, y, w, l
    right_roi = (510, 200, 160, 180)

    board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
    board.set_rgb([[1, 255, 255, 0]]) #YELLOW
    time.sleep(5)

    print("Board initialized")

    """
    switch = 17

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(switch, GPIO.OUT)


    while GPIO.input(switch) == GPIO.HIGH:
        pass
    """

    board.set_rgb([[1, 0, 0, 0]]) #OFF
    print("Starting")
    time.sleep(1)
    board.set_rgb([[1, 0, 255, 0]]) #GREEN
    board.set_rgb([[2, 0, 255, 0]]) #GREEN
    
    
    # === Main loop ===
    while not lap_complete:
        # ---- Get camera frame ----
        frame = picam2.capture_array()
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
            print("Turning left")
        elif right_area <= TURN_THRESHOLD and not left_turn:
            right_turn = True
            print("Turning right")

        if left_turn or right_turn:
            if (right_area >= EXIT_THRESHOLD and right_turn) or (left_area >= EXIT_THRESHOLD and left_turn):
                left_turn = right_turn = False
                turns += 1
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
                                                                
        prev_diff = area_diff
        prev_angle = angle_pwm



        # ---- Stop ----
        if turns == 12: 
            print("Completed 3 laps. Stopping.")
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
