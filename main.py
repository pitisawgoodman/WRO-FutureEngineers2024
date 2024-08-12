import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time

# GPIO Setup
GPIO.setmode(GPIO.BCM)

# Motor and Servo Pins
MOTOR_PIN1 = 17
MOTOR_PIN2 = 27
SERVO_PIN = 18
ENA_PIN = 12  # PWM pin for enabling motor speed

GPIO.setup(MOTOR_PIN1, GPIO.OUT)
GPIO.setup(MOTOR_PIN2, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(ENA_PIN, GPIO.OUT)

# Initialize PWM for Servo
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(7.5)  # Neutral position

# Initialize PWM for Motor Speed Control
motor_pwm = GPIO.PWM(ENA_PIN, 100)  # 100Hz PWM frequency
motor_pwm.start(0)  # Start with 0% duty cycle (motor off)

# Ultrasonic Sensor Pins
FRONT_TRIG = 23
FRONT_ECHO = 24
LEFT_TRIG = 20
LEFT_ECHO = 21
RIGHT_TRIG = 5
RIGHT_ECHO = 6
BACK_TRIG = 22
BACK_ECHO = 23

GPIO.setup(FRONT_TRIG, GPIO.OUT)
GPIO.setup(FRONT_ECHO, GPIO.IN)
GPIO.setup(LEFT_TRIG, GPIO.OUT)
GPIO.setup(LEFT_ECHO, GPIO.IN)
GPIO.setup(RIGHT_TRIG, GPIO.OUT)
GPIO.setup(RIGHT_ECHO, GPIO.IN)
GPIO.setup(BACK_TRIG, GPIO.OUT)
GPIO.setup(BACK_ECHO, GPIO.IN)

# Define Traffic Light Colors
red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])
yellow_lower = np.array([18, 100, 100])
yellow_upper = np.array([30, 255, 255])
green_lower = np.array([45, 100, 100])
green_upper = np.array([75, 255, 255])

# Initialize Camera
cap = cv.VideoCapture(0)

def measure_distance(trig_pin, echo_pin, timeout=0.1):
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    pulse_start = time.time()
    while GPIO.input(echo_pin) == 0:
        if time.time() - pulse_start > timeout:
            return None

    pulse_start = time.time()
    while GPIO.input(echo_pin) == 1:
        if time.time() - pulse_start > timeout:
            return None

    pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

def drive_motor(direction, speed=100):
    # Speed is a percentage (0-100) of the maximum PWM duty cycle
    motor_pwm.ChangeDutyCycle(speed)
    
    if direction == 'forward':
        GPIO.output(MOTOR_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR_PIN2, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(MOTOR_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_PIN2, GPIO.HIGH)
    elif direction == 'stop':
        GPIO.output(MOTOR_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_PIN2, GPIO.LOW)
        motor_pwm.ChangeDutyCycle(0)  # Ensure motor is off

def turn_servo(direction):
    if direction == 'left':
        servo.ChangeDutyCycle(5)
    elif direction == 'right':
        servo.ChangeDutyCycle(10)
    elif direction == 'straight':
        servo.ChangeDutyCycle(7.5)

def detect_traffic_light(frame):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    red_mask = cv.inRange(hsv, red_lower, red_upper)
    yellow_mask = cv.inRange(hsv, yellow_lower, yellow_upper)
    green_mask = cv.inRange(hsv, green_lower, green_upper)

    contours_red, _ = cv.findContours(red_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_yellow, _ = cv.findContours(yellow_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv.findContours(green_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    if contours_red:
        return 'red'
    elif contours_yellow:
        return 'yellow'
    elif contours_green:
        return 'green'
    return None

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv.resize(frame, (600, 600))
        traffic_light = detect_traffic_light(frame)

        # Measure distances from all ultrasonic sensors
        front_distance = measure_distance(FRONT_TRIG, FRONT_ECHO)
        left_distance = measure_distance(LEFT_TRIG, LEFT_ECHO)
        right_distance = measure_distance(RIGHT_TRIG, RIGHT_ECHO)
        back_distance = measure_distance(BACK_TRIG, BACK_ECHO)

        cv.putText(frame, f'Front Distance: {front_distance} cm', (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv.putText(frame, f'Left Distance: {left_distance} cm', (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv.putText(frame, f'Right Distance: {right_distance} cm', (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv.putText(frame, f'Back Distance: {back_distance} cm', (10, 120), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if traffic_light == 'red':
            drive_motor('stop')
            cv.putText(frame, 'Red Light Detected', (10, 150), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        elif traffic_light == 'yellow':
            drive_motor('stop')
            cv.putText(frame, 'Yellow Light Detected', (10, 150), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        elif traffic_light == 'green':
            cv.putText(frame, 'Green Light Detected', (10, 150), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Obstacle Avoidance Logic
        if front_distance is not None and front_distance < 20:
            drive_motor('stop')
            cv.putText(frame, 'Obstacle Detected Ahead!', (10, 180), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        elif left_distance < 15 and right_distance >= 15:
            turn_servo('right')
            time.sleep(1)
            turn_servo('straight')
        elif right_distance < 15 and left_distance >= 15:
            turn_servo('left')
            time.sleep(1)
            turn_servo('straight')
        elif back_distance is not None and back_distance < 15:
            drive_motor('stop')
            cv.putText(frame, 'Obstacle Detected Behind!', (10, 180), cv.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
        else:
            if traffic_light != 'red':
                drive_motor('forward')

        cv.imshow('Self-Driving Car', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv.destroyAllWindows()
    drive_motor('stop')
    turn_servo('straight')
    servo.stop()
    motor_pwm.stop()
    GPIO.cleanup()
