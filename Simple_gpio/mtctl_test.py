from ekagaku_motor import MotorCtl
import time


def main():
    motor = MotorCtl()
    motor.forward(50)
    time.sleep(2)
    motor.backward(50)
    time.sleep(2)
    motor.left(50)
    time.sleep(2)
    motor.right(50)
    time.sleep(2)
    motor.stop()

if __name__ == '__main__':
    main()