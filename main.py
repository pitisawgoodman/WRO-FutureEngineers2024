import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time

# Setup GPIO
GPIO.setmode(GPIO.BCM)

# Motor control setup
MOTOR_PIN1 = 17
MOTOR_PIN2 = 27
GPIO.setup(MOTOR_PIN1, GPIO.OUT)
GPIO.setup(MOTOR_PIN2, GPIO.OUT)

# Servo motor setup
SERVO_PIN = 18  # PWM pin for the servo
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM frequency
servo.start(7.5)  # Neutral position

# Ultrasonic sensor setup
FRONT_TRIG = 23
FRONT_ECHO = 24
LEFT_TRIG = 20
LEFT_ECHO = 21
RIGHT_TRIG = 5
RIGHT_ECHO = 6

GPIO.setup(FRONT_TRIG, GPIO.OUT)
GPIO.setup(FRONT_ECHO, GPIO.IN)
GPIO.setup(LEFT_TRIG, GPIO.OUT)
GPIO.setup(LEFT_ECHO, GPIO.IN)
GPIO.setup(RIGHT_TRIG, GPIO.OUT)
GPIO.setup(RIGHT_ECHO, GPIO.IN)

# Define color ranges for traffic light detection
red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])

yellow_lower = np.array([18, 100, 100])
yellow_upper = np.array([30, 255, 255])

green_lower = np.array([45, 100, 100])
green_upper = np.array([75, 255, 255])

# Function to measure distance
def measure_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)
    
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
    
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

# Function to control the drive motor
def drive_motor(direction):
    if direction == 'forward':
        GPIO.output(MOTOR_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR_PIN2, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(MOTOR_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_PIN2, GPIO.HIGH)
    elif direction == 'stop':
        GPIO.output(MOTOR_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_PIN2, GPIO.LOW)

# Function to control the servo motor for steering
def turn_servo(direction):
    if direction == 'left':
        servo.ChangeDutyCycle(5)  # Turn left
    elif direction == 'right':
        servo.ChangeDutyCycle(10)  # Turn right
    elif direction == 'straight':
        servo.ChangeDutyCycle(7.5)  # Neutral position

# Initialize webcam
cap = cv.VideoCapture(0)

try:
    while True:
        # Capture frame from webcam
        ret, frame = cap.read()
        if not ret:
            break
        
        # Resize the frame for faster processing
        frame = cv.resize(frame, (600, 600))

        # Convert the frame to HSV color space
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Create masks for red, yellow, and green colors
        red_mask = cv.inRange(hsv, red_lower, red_upper)
        yellow_mask = cv.inRange(hsv, yellow_lower, yellow_upper)
        green_mask = cv.inRange(hsv, green_lower, green_upper)

        # Find contours for each color
        contours_red, _ = cv.findContours(red_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv.findContours(yellow_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv.findContours(green_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Traffic light detection logic
        light_detected = False
        if contours_red:
            for contour in contours_red:
                area = cv.contourArea(contour)
                if area > 500:
                    x, y, w, h = cv.boundingRect(contour)
                    cv.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv.putText(frame, 'Red Light', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                    drive_motor('stop')
                    light_detected = True
                    break

        if contours_yellow and not light_detected:
            for contour in contours_yellow:
                area = cv.contourArea(contour)
                if area > 500:
                    x, y, w, h = cv.boundingRect(contour)
                    cv.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
                    cv.putText(frame, 'Yellow Light', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
                    drive_motor('stop')
                    light_detected = True
                    break

        if contours_green and not light_detected:
            for contour in contours_green:
                area = cv.contourArea(contour)
                if area > 500:
                    x, y, w, h = cv.boundingRect(contour)
                    cv.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv.putText(frame, 'Green Light', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    light_detected = True
                    break

        # Measure distances from ultrasonic sensors
        front_distance = measure_distance(FRONT_TRIG, FRONT_ECHO)
        left_distance = measure_distance(LEFT_TRIG, LEFT_ECHO)
        right_distance = measure_distance(RIGHT_TRIG, RIGHT_ECHO)

        # Display ultrasonic sensor values on the left side of the screen
        cv.putText(frame, f'Front Distance: {front_distance} cm', (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv.putText(frame, f'Left Distance: {left_distance} cm', (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv.putText(frame, f'Right Distance: {right_distance} cm', (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Obstacle avoidance logic
        if front_distance < 20:
            drive_motor('stop')
            print("Obstacle detected ahead! Stopping.")
        elif left_distance < 15 and right_distance >= 15:
            print("Obstacle detected on the left! Turning right.")
            turn_servo('right')
            time.sleep(1)
            turn_servo('straight')
        elif right_distance < 15 and left_distance >= 15:
            print("Obstacle detected on the right! Turning left.")
            turn_servo('left')
            time.sleep(1)
            turn_servo('straight')
        else:
            if not light_detected:
                drive_motor('forward')
        
        # Display the frame
        cv.imshow('Traffic Light Detection', frame)

        # Break loop on 'q' key press
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Clean up
    cap.release()
    cv.destroyAllWindows()
    drive_motor('stop')
    turn_servo('straight')
    servo.stop()
    GPIO.cleanup()
