import cv2
import numpy as np
import time
from picamera2 import Picamera2
import ros_robot_controller_sdk as rrc

if __name__ == '__main__':
    time.sleep(1)
    # === CONFIGURATION ===
    STRAIGHT_PWM = 1500
    THROTTLE_PWM = 1680
    THROTTLE_TURN = 1680
    TURN_THRESHOLD = 2500
    EXIT_THRESHOLD = 9000
    MAX_LEFT = 1650
    MAX_RIGHT = 1350
    TURN_DEV = 50
    board = rrc.Board()
    # === STATE VARIABLES ===
    last_turn_time = time.time()
    prev_diff = 0
    prev_angle = STRAIGHT_PWM
    left_turn = False
    right_turn = False

    turns = 0
    lap_complete = False
    
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

    while not lap_complete:
        frame = picam2.capture_array()
        # Thresholding
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)


        # Crop ROIs
        left_crop = thresh[left_roi[1]:left_roi[1]+left_roi[3], left_roi[0]:left_roi[0]+left_roi[2]]
        right_crop = thresh[right_roi[1]:right_roi[1]+right_roi[3], right_roi[0]:right_roi[0]+right_roi[2]]

        # Count black pixels
        left_area = cv2.countNonZero(left_crop)
        right_area = cv2.countNonZero(right_crop)

        angle_pwm = 1500

        # Turn state logic
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
                                                                
        prev_angle = angle_pwm


        if turns == 12:
            print("Completed 3 laps. Stopping.")
            board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
            lap_complete = True

        # Debug view
        cv2.rectangle(frame, (left_roi[0], left_roi[1]), (left_roi[0]+left_roi[2], left_roi[1]+left_roi[3]), (255, 0, 0), 2)
        cv2.rectangle(frame, (right_roi[0], right_roi[1]), (right_roi[0]+right_roi[2], right_roi[1]+right_roi[3]), (0, 0, 255), 2)
        cv2.imshow("Live View", frame)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
            break

    cv2.destroyAllWindows()
