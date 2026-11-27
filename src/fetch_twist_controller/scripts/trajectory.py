#!/usr/bin/env python3
import argparse
import csv
import math
import time
import yaml
import rospy
from geometry_msgs.msg import TwistStamped

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('file'); p.add_argument('--frame',default=None)
    p.add_argument('--rate',type=float,default=100.); args=p.parse_args(rospy.myargv()[1:])
    with open(args.file) as f:
        if args.file.endswith('.csv'):
            steps=[dict(duration=float(r['duration']),frame=r.get('frame','base_link'),
                        twist=[float(r[k]) for k in ('vx','vy','vz','wx','wy','wz')]) for r in csv.DictReader(f)]
        else: steps=yaml.safe_load(f)['steps']
    for s in steps:
        if len(s['twist'])!=6 or not all(math.isfinite(float(v)) for v in s['twist']) or not math.isfinite(float(s['duration'])) or float(s['duration'])<=0:
            raise SystemExit('Invalid trajectory segment')
    if not math.isfinite(args.rate) or args.rate<=0: raise SystemExit('Invalid rate')
    rospy.init_node('twist_trajectory'); pub=rospy.Publisher('/fetch_twist/command',TwistStamped,queue_size=1)
    def send(v,frame):
        m=TwistStamped(); m.header.stamp=rospy.Time.now(); m.header.frame_id=frame
        m.twist.linear.x,m.twist.linear.y,m.twist.linear.z=v[:3]
        m.twist.angular.x,m.twist.angular.y,m.twist.angular.z=v[3:]; pub.publish(m)
    deadline=time.monotonic()+5
    while pub.get_num_connections()==0 and time.monotonic()<deadline: time.sleep(.05)
    if pub.get_num_connections()==0: raise SystemExit('No twist subscriber; start control.launch')
    frame=args.frame or 'base_link'
    try:
        for step in steps:
            frame=args.frame or step.get('frame','base_link')
            start=rospy.Time.now().to_sec(); end=start+float(step['duration'])
            last_sim=start; last_progress=time.monotonic()
            while not rospy.is_shutdown() and rospy.Time.now().to_sec()<end:
                now=rospy.Time.now().to_sec()
                if now<last_sim: raise RuntimeError('Simulation clock moved backwards')
                if now>last_sim: last_progress=time.monotonic(); last_sim=now
                if time.monotonic()-last_progress>1: raise RuntimeError('Simulation paused: trajectory stopped')
                send(step['twist'],frame); time.sleep(1./args.rate)
    finally:
        for _ in range(5): send([0.]*6,frame); time.sleep(.01)
