from ekagaku_motor import MotorCtl
import time


def main():
    m=MotorCtl()
    # w: forward, s: backward, a: left, d: right, x: stop e"speed up, c: speed down
    pwL=50
    pwR=50
    while True:
        key=input('command:')
        if key=='w':
            m.motor(pwL, pwR)
        elif key=='s':
            m.motor(-pwL, -pwR)
        elif key=='a':
            m.motor(-pwL, pwR)
        elif key=='d':
            m.motor(pwL, -pwR)
        elif key=='x':
            m.stop()
        elif key=='e':
            pwL=min(100, pwL+10)
            pwR=min(100, pwR+10)
            print('pwL:', pwL, 'pwR:', pwR)
        elif key=='c':
            pwL=max(-100, pwL-10)
            pwR=max(-100, pwR-10)
            print('pwL:', pwL, 'pwR:', pwR)
        else:
            break
        time.sleep(0.1)
        



        

