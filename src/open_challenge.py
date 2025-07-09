import cv2
import numpy as np
import time
from picamera2 import Picamera2
import ros_robot_controller_sdk as rrc


# Constants
STRAIGHT_PWM = 1500
LEFT_TURN_PWM = 1600
RIGHT_TURN_PWM = 1400
SPEED_PWM = 1650
TURN_THRESHOLD = 3500
MAX_LEFT = 1620
MAX_RIGHT = 1380
TURN_DEV = 30
turns = 0
lap_complete = False
board = rrc.Board()

# Initialize camera
board = rrc.Board()
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.configure("preview")
picam2.start()
time.sleep(2)


# Define ROIs
left_roi = (0, 220, 130, 60) 
right_roi = (510, 220, 130, 60)

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

    # Turn state logic
    if left_area <= TURN_THRESHOLD and not right_turn:
        angle_pwm = LEFT_TURN_PWM
        left_turn = True
        print("Turning left")
    elif right_area <= TURN_THRESHOLD and not left_turn:
        angle_pwm = RIGHT_TURN_PWM
        right_turn = True
        print("Turning right")
    else:
        angle_pwm = 1500
        left_turn = right_turn = False

        # ---- Drive ----
        if left_turn or right_turn:
            board.pwm_servo_set_position(0.1, [[4, angle_pwm], [2, 1600]])
        else:
            board.pwm_servo_set_position(0.1, [[4, angle_pwm], [2, SPEED_PWM]])
        
        # Add turn stop condition


    # Debug view
    cv2.rectangle(frame, (left_roi[0], left_roi[1]), (left_roi[0]+left_roi[2], left_roi[1]+left_roi[3]), (255, 0, 0), 2)
    cv2.rectangle(frame, (right_roi[0], right_roi[1]), (right_roi[0]+right_roi[2], right_roi[1]+right_roi[3]), (0, 0, 255), 2)
    cv2.imshow("Live View", frame)

    # Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        board.pwm_servo_set_position(0.1, [[4, 1500], [2, 1500]])
        break

cv2.destroyAllWindows()
