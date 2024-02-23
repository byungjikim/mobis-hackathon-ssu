# 조이스틱의 입력을 모터에 적용하기 위한 함수. = joy.py와 함께 사용하는 함수
# -*- coding: utf-8 -*-

# 라즈베리파이 GPIO 패키지 
import RPi.GPIO as GPIO
import time
import board
import busio
from adafruit_pca9685 import PCA9685# pip install adafruit-circuitpython-pca9685
from adafruit_motor import servo
# 모터 상태
STOP  = 0
FORWARD  = 1
BACKWARD = 2

# 모터 채널
CH1 = 0
CH2 = 1

# PIN 입출력 설정
OUTPUT = 1
INPUT = 0

# PIN 설정
HIGH = 1
LOW = 0

# 실제 핀 정의
# 19 
RIGHT_EN = 17
RIGHT_F = 27
RIGHT_B = 18

# 4 26 21
# 17 27 18
LEFT_EN = 4
LEFT_F = 26
LEFT_B = 21
# 서보 모터
HANDLE_PIN = 18

#모터 클래스
class Motor:
    def __init__(self):        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LEFT_EN, GPIO.OUT)
        GPIO.setup(LEFT_F, GPIO.OUT)
        GPIO.setup(LEFT_B, GPIO.OUT)
        GPIO.setup(RIGHT_EN, GPIO.OUT)
        GPIO.setup(RIGHT_F, GPIO.OUT)
        GPIO.setup(RIGHT_B, GPIO.OUT)
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.left_pwm = GPIO.PWM(LEFT_EN, 100)
        self.right_pwm = GPIO.PWM(RIGHT_EN, 100)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        pca = PCA9685(i2c_bus)
        pca.frequency = 50
        servo_pin = 10
        self.servo_mt = servo.Servo(pca.channels[servo_pin])
       
    def motor_speed(self, speed):
        self.left_pwm.ChangeDutyCycle(speed)
        self.right_pwm.ChangeDutyCycle(speed)

    def move_forward(self):
            GPIO.output(LEFT_F, GPIO.HIGH)
            GPIO.output(LEFT_B, GPIO.LOW)
            GPIO.output(RIGHT_F, GPIO.HIGH)
            GPIO.output(RIGHT_B, GPIO.LOW)
    
    def move_backward(self):
            GPIO.output(LEFT_F, GPIO.LOW)
            GPIO.output(LEFT_B, GPIO.HIGH)
            GPIO.output(RIGHT_F, GPIO.LOW)
            GPIO.output(RIGHT_B, GPIO.HIGH)
    
    def stop_move(self):
            GPIO.output(LEFT_F, GPIO.LOW)
            GPIO.output(LEFT_B, GPIO.LOW)
            GPIO.output(RIGHT_F, GPIO.LOW)
            GPIO.output(RIGHT_B, GPIO.LOW)
    
    
    def move_servo(self, angle):
            self.servo_mt.angle=angle

    
    def __del__(self):
        GPIO.cleanup()

# 테스트로 돌려보기
#if __name__ == "__main__":
#        mt = Motor()
#        mt.motor_speed(30)
#        mt.move_forward()
#        time.sleep(3)
#        mt.motor_speed(0)

