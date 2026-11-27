#!/usr/bin/env python3
"""Measure joint-state message arrival gaps over 15 seconds."""

import threading
import time

import rospy
from sensor_msgs.msg import JointState

lock = threading.Lock()
last = time.monotonic()
gaps = []
count = 0


def received(msg):
    global last, count
    now = time.monotonic()
    with lock:
        if count:
            gaps.append(now - last)
        last = now
        count += 1


rospy.init_node("check_feedback_timing", anonymous=True)
sub = rospy.Subscriber("/joint_states", JointState, received, queue_size=1)

print("Measuring joint-state arrivals for 15 seconds...")
time.sleep(15)

with lock:
    print("Messages received:", count)
    print("Largest arrival gap:", round(max(gaps, default=0), 3), "seconds")
    print("Gaps over 0.2 seconds:", sum(g > 0.2 for g in gaps))
    print("Time since last arrival:", round(time.monotonic() - last, 3), "seconds")
