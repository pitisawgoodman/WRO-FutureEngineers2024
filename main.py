import RPi.GPIO as GPIO
import time
from gpiozero import Servo, Button

# ตั้งค่า GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# กำหนด GPIO พินสำหรับ L298N Motor Driver (มอเตอร์ตัวเดียว)
in1 = 24
in2 = 23
enA = 25

# ตั้งค่า GPIO พินเป็น Output
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(enA, GPIO.OUT)

# ตั้งค่าความถี่ PWM และสร้าง object PWM
pwmA = GPIO.PWM(enA, 1000)  # ความถี่ 1kHz สำหรับ Motor A
pwmA.start(0)  # เริ่มต้น PWM ด้วยค่า duty cycle ที่ 0

# กำหนดพินเซอร์โว
servo_pin = 22  # ใช้พิน GPIO 22
servo = Servo(servo_pin)

# กำหนดพินปุ่ม
button = Button(2)

# กำหนด GPIO พินสำหรับเซ็นเซอร์แต่ละทิศทาง
sensors = {
    "front": {"trigger": 21, "echo": 20},
    "back": {"trigger": 17, "echo": 27},
    "left": {"trigger": 6, "echo": 5},
    "right": {"trigger": 26, "echo": 19}
}

# ตั้งค่า GPIO พิน เป็น Input และ Output สำหรับเซ็นเซอร์
for direction, sensor in sensors.items():
    GPIO.setup(sensor["trigger"], GPIO.OUT)
    GPIO.setup(sensor["echo"], GPIO.IN)

def distance(sensor):
    """วัดระยะทางจากเซ็นเซอร์"""
    GPIO.output(sensor["trigger"], True)
    time.sleep(0.001)
    GPIO.output(sensor["trigger"], False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(sensor["echo"]) == 0:
        start_time = time.time()

    while GPIO.input(sensor["echo"]) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2
    
    return distance

def move_forward(speed):
    """เคลื่อนที่ไปข้างหน้า"""
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    pwmA.ChangeDutyCycle(speed)

def move_backward(speed):
    """ถอยหลัง"""
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    pwmA.ChangeDutyCycle(speed)

def stop():
    """หยุดการเคลื่อนไหว"""
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    pwmA.ChangeDutyCycle(0)

def turn_servo_to_position(percentage):
    """ปรับตำแหน่งเซอร์โวตามเปอร์เซ็นต์"""
    position = percentage / 100.0
    servo.value = position

def keep_center():
    """ปรับตำแหน่งเซอร์โวเพื่อให้รถอยู่กลางเลน"""
    left_dist = distance(sensors["left"])
    right_dist = distance(sensors["right"])

    if left_dist < 100 and right_dist < 100:
        if left_dist > right_dist:
            turn_servo_to_position(-50)  # เลี้ยวซ้าย
        elif right_dist > left_dist:
            turn_servo_to_position(50)  # เลี้ยวขวา
        else:
            turn_servo_to_position(0)  # อยู่กลาง
    elif left_dist < 100:
        turn_servo_to_position(-50)  # เลี้ยวซ้าย
    elif right_dist < 100:
        turn_servo_to_position(50)  # เลี้ยวขวา

def avoid_obstacle():
    """ฟังก์ชันหลักสำหรับหลบสิ่งกีดขวางและตัดสินใจ"""
    while system_running:
        front_dist = distance(sensors["front"])
        back_dist = distance(sensors["back"])

        # ตรวจสอบระยะทางด้านหน้าและด้านหลัง
        if front_dist < 100:
            stop()
            time.sleep(0.5)

            # ถอยหลัง
            move_backward(50)
            time.sleep(1)
            stop()
            time.sleep(0.5)

            while front_dist < 100:
                back_dist = distance(sensors["back"])
                front_dist = distance(sensors["front"])

                if back_dist < 20:
                    turn_servo_to_position(50)  # เลี้ยวขวา
                    move_forward(20)
                    time.sleep(1)
                else:
                    left_dist = distance(sensors["left"])
                    right_dist = distance(sensors["right"])

                    if left_dist > right_dist:
                        turn_servo_to_position(-50)  # เลี้ยวซ้าย
                        move_forward(50)
                        time.sleep(1)
                    else:
                        turn_servo_to_position(50)  # เลี้ยวขวา
                        move_forward(20)
                        time.sleep(1)

                front_dist = distance(sensors["front"])
                stop()
                time.sleep(0.5)
            
            move_forward(40)
            time.sleep(1)
        else:
            move_forward(40)

        keep_center()

def toggle_system():
    """สลับการทำงานของระบบเมื่อกดปุ่ม"""
    global system_running
    if system_running:
        stop()
        system_running = False
        print("ระบบหยุดทำงาน")
    else:
        system_running = True
        print("ระบบเริ่มทำงาน")

# กำหนดให้เมื่อกดปุ่มจะเปลี่ยนสถานะการทำงาน
button.when_pressed = toggle_system

# สถานะเริ่มต้น
system_running = False

try:
    print("กดปุ่มเพื่อเริ่มหรือหยุดระบบ")

    while True:
        if system_running:
            avoid_obstacle()
        else:
            time.sleep(0.1)

except KeyboardInterrupt:
    print("การทำงานหยุดโดยผู้ใช้")
    stop()
    GPIO.cleanup()
