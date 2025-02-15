#control motor by /cmd_vel topic
#subscribe /cmd_vel topic 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import time

import RPi.GPIO as GPIO

class MotorCtl:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.ENA = 13
        self.IN1 = 19
        self.IN2 = 26
        self.IN3 = 20
        self.IN4 = 21
        self.ENB = 16
        GPIO.setup(self.ENA, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN4, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ENB, GPIO.OUT, initial=GPIO.LOW)
        self.pwm_L = GPIO.PWM(self.ENA, 1000)
        self.pwm_R = GPIO.PWM(self.ENB, 1000)
        self.pwm_L.start(0)
        self.pwm_R.start(0)

    # def forward(self, speed):
    #     self.pwm_L.ChangeDutyCycle(speed)
    #     self.pwm_R.ChangeDutyCycle(speed)
    #     GPIO.output(self.IN1, GPIO.HIGH)
    #     GPIO.output(self.IN2, GPIO.LOW)
    #     GPIO.output(self.IN3, GPIO.HIGH)
    #     GPIO.output(self.IN4, GPIO.LOW)

    # def backward(self, speed):
    #     self.pwm_L.ChangeDutyCycle(speed)
    #     self.pwm_R.ChangeDutyCycle(speed)
    #     GPIO.output(self.IN1, GPIO.LOW)
    #     GPIO.output(self.IN2, GPIO.HIGH)
    #     GPIO.output(self.IN3, GPIO.LOW)
    #     GPIO.output(self.IN4, GPIO.HIGH)

    # def left(self, speed):
    #     self.pwm_L.ChangeDutyCycle(speed)
    #     self.pwm_R.ChangeDutyCycle(speed)
    #     GPIO.output(self.IN1, GPIO.LOW)
    #     GPIO.output(self.IN2, GPIO.HIGH)
    #     GPIO.output(self.IN3, GPIO.HIGH)
    #     GPIO.output(self.IN4, GPIO.LOW)

    # def right(self, speed):
    #     self.pwm_L.ChangeDutyCycle(speed)
    #     self.pwm_R.ChangeDutyCycle(speed)
    #     GPIO.output(self.IN1, GPIO.HIGH)
    #     GPIO.output(self.IN2, GPIO.LOW)
    #     GPIO.output(self.IN3, GPIO.LOW)
        
    def stop(self):
        self.pwm_L.ChangeDutyCycle(0)
        self.pwm_R.ChangeDutyCycle(0)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
                    
    def motor(self, speed_L, speed_R):
        self.pwm_L.ChangeDutyCycle(speed_L)
        self.pwm_R.ChangeDutyCycle(speed_R)
        if speed_L > 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        if speed_R > 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
    
    
    
    def __del__(self):
        self.pwm_L.stop()
        self.pwm_R.stop()
        GPIO.cleanup()
        
        
            
            
class RaspiMotorCtl(Node):
    
    def __init__(self):
        super().__init__('raspi_motor_ctl')
        self.motor_ctl = MotorCtl()
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Subscribed to /cmd_vel topic")

    def listener_callback(self, msg):
        self.get_logger().info('Linear: %f, Angular: %f' % (msg.linear.x, msg.angular.z))
        speed = msg.linear.x
        angular = msg.angular.z

        if speed == 0 and angular == 0:
            self.motor_ctl.stop()
        else:
            speed_L = speed + angular
            speed_R = speed - angular
            self.motor_ctl.motor(speed_L, speed_R)

def main(args=None):
    rclpy.init(args=args)

    raspi_motor_ctl = RaspiMotorCtl()

    rclpy.spin(raspi_motor_ctl)

    raspi_motor_ctl.destroy_node()
    rclpy.shutdown()



        