#control motor by /cmd_vel topic
#subscribe /cmd_vel topic and publish /wheel_speed topic

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
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
        #if speed is out of range, warning and set to max/min value
        if (speed_L > 100 or speed_L < -100)or (speed_R > 100 or speed_R < -100):
            print("Speed out of range")
        speed_L = min(100, max(-100, speed_L))
        speed_R = min(100, max(-100, speed_R))

        self.pwm_L.ChangeDutyCycle(abs(speed_L))
        self.pwm_R.ChangeDutyCycle(abs(speed_R))
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
        self.speed_L = 0
        self.speed_R = 0
        self.wheel_radius = 0.025
        self.max_rpm = 130

        self.motor_ctl = MotorCtl()
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.speed_pub = self.create_publisher(Float64MultiArray, 'wheel_speed', 10)
        self.timer= self.create_timer(0.1, self.timer_cb)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Subscribed to /cmd_vel topic")

        

    def listener_callback(self, msg):
        self.get_logger().info('Linear: %f, Angular: %f' % (msg.linear.x, msg.angular.z))
        speed = msg.linear.x
        angular = msg.angular.z

        if speed == 0 and angular == 0:
            self.motor_ctl.stop()
            self.speed_L = 0
            self.speed_R = 0
        else:
            self.speed_L = speed - angular
            self.speed_R = speed + angular
            #calculate wheel speed in rpm
            self.speed_L = self.speed_L * 60 / (2 * math.pi * self.wheel_radius)
            self.speed_R = self.speed_R * 60 / (2 * math.pi * self.wheel_radius)
            #convert wheel speed to pwm
            self.speed_L = self.speed_L * 100 / self.max_rpm
            self.speed_R = self.speed_R * 100 / self.max_rpm
            
            self.motor_ctl.motor(self.speed_L, self.speed_R)

        
    def timer_cb(self):
        speed_msg = Float64MultiArray()
        speed_msg.data = [float(self.speed_L), float(self.speed_R)]
        self.get_logger().info('Speed L: %f, Speed R: %f' % (self.speed_L, self.speed_R))
        self.speed_pub.publish(speed_msg)
        
        

def main(args=None):
    rclpy.init(args=args)

    raspi_motor_ctl = RaspiMotorCtl()

    rclpy.spin(raspi_motor_ctl)

    raspi_motor_ctl.destroy_node()
    rclpy.shutdown()



        