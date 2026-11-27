#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import time
import tty
import rospy
from geometry_msgs.msg import TwistStamped

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--frame',default='base_link')
    p.add_argument('--linear',type=float,default=0.02); p.add_argument('--angular',type=float,default=0.1)
    args=p.parse_args(rospy.myargv()[1:]); rospy.init_node('twist_keyboard')
    pub=rospy.Publisher('/fetch_twist/command',TwistStamped,queue_size=1)
    bindings={'w':(0,1),'s':(0,-1),'a':(1,1),'d':(1,-1),'r':(2,1),'f':(2,-1),
              'u':(3,1),'j':(3,-1),'i':(4,1),'k':(4,-1),'o':(5,1),'l':(5,-1)}
    if not sys.stdin.isatty(): raise SystemExit('Run keyboard.py in an interactive terminal')
    print('w/s: x, a/d: y, r/f: z; u/j: roll, i/k: pitch, o/l: yaw')
    print('SPACE: stop; 1: base_link; 2: gripper_link; 3: user_frame; Ctrl-C: exit')
    print('Terminal keys have no key-up events: motion expires 0.2 s after the last key repeat.')
    old=termios.tcgetattr(sys.stdin); values=[0.]*6; last=0.; frame=args.frame
    def send(v):
        m=TwistStamped(); m.header.stamp=rospy.Time.now(); m.header.frame_id=frame
        m.twist.linear.x,m.twist.linear.y,m.twist.linear.z=v[:3]
        m.twist.angular.x,m.twist.angular.y,m.twist.angular.z=v[3:]; pub.publish(m)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            if select.select([sys.stdin],[],[],0.01)[0]:
                key=sys.stdin.read(1)
                if key=='\x03': break
                values=[0.]*6; last=time.monotonic()
                if key in bindings:
                    axis,sign=bindings[key]; values[axis]=sign*(args.linear if axis<3 else args.angular)
                if key in ('1','2','3'): frame={'1':'base_link','2':'gripper_link','3':'user_frame'}[key]
            if time.monotonic()-last>0.2: values=[0.]*6
            send(values)
    finally:
        send([0.]*6); time.sleep(0.05); termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old)
