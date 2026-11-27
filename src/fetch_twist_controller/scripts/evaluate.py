#!/usr/bin/env python3
"""Compare measured J(q)*qdot to latest target at or before measurement time."""
import argparse
import csv
import os
import threading
from collections import deque
import rospy
from geometry_msgs.msg import TwistStamped

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--output',required=True); args=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('twist_evaluation'); os.makedirs(os.path.dirname(os.path.abspath(args.output)),exist_ok=True)
    f=open(args.output,'w',newline=''); w=csv.writer(f); lock=threading.Lock(); history=deque(maxlen=2000)
    labels=['vx','vy','vz','wx','wy','wz']; last=[-1.]
    w.writerow(['time','frame','target_age']+['command_'+k for k in labels]+['measured_'+k for k in labels]); f.flush()
    def values(m):
        t=m.twist; return [t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z]
    def target(m):
        with lock:
            if history and m.header.stamp<history[-1].header.stamp: history.clear(); last[0]=-1.
            history.append(m)
    def measured(m):
        with lock:
            stamp=m.header.stamp.to_sec()
            if stamp<=last[0]: return
            eligible=[c for c in history if c.header.stamp<=m.header.stamp and c.header.frame_id==m.header.frame_id]
            if not eligible: return
            c=eligible[-1]; age=stamp-c.header.stamp.to_sec()
            if age>0.1: return
            w.writerow([stamp,m.header.frame_id,age]+values(c)+values(m)); f.flush(); last[0]=stamp
    rospy.Subscriber('/twist_controller/target_twist',TwistStamped,target,queue_size=200)
    rospy.Subscriber('/twist_controller/measured_twist',TwistStamped,measured,queue_size=200)
    rospy.loginfo('Recording %s; Ctrl-C finishes recording',args.output)
    try: rospy.spin()
    finally: f.close()
