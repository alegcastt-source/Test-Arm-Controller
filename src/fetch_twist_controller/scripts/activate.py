#!/usr/bin/env python3
"""Switch Fetch's action-based manager to/from the joint velocity plugin."""
import argparse
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from robot_controllers_msgs.msg import QueryControllerStatesAction,QueryControllerStatesGoal,ControllerState

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--stop',action='store_true'); args=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('activate_twist_plugin')
    if not rospy.get_param('/use_sim_time',False):
        raise SystemExit('This launcher is intended for Gazebo: /use_sim_time must be true.')
    c=actionlib.SimpleActionClient('/query_controller_states',QueryControllerStatesAction)
    if not c.wait_for_server(rospy.Duration(10)): raise SystemExit('Fetch controller manager not found')
    def request(updates):
        c.send_goal(QueryControllerStatesGoal(updates=updates))
        if not c.wait_for_result(rospy.Duration(10)): raise SystemExit('Controller switch timed out')
        if c.get_state()!=GoalStatus.SUCCEEDED: raise SystemExit(c.get_goal_status_text())
        return c.get_result()
    def entry(name,state): return ControllerState(name=name,state=state)
    if args.stop:
        request([entry('fetch_joint_velocity',ControllerState.STOPPED)])
        print('Velocity plugin stopped. Restart Fetch simulation to restore its initial controller configuration.')
    else:
        result=request([])
        conflicts=('arm_controller/follow_joint_trajectory','arm_with_torso_controller/follow_joint_trajectory')
        updates=[entry(s.name,ControllerState.STOPPED) for s in result.state
                 if s.name in conflicts and s.state==ControllerState.RUNNING]
        # Stops are applied before start in one action. No movement command is sent here.
        updates.append(entry('fetch_joint_velocity',ControllerState.RUNNING))
        result=request(updates)
        if not any(s.name=='fetch_joint_velocity' and s.state==ControllerState.RUNNING for s in result.state):
            raise SystemExit('Velocity plugin did not enter RUNNING')
        print('Velocity plugin running; output stays zero until fresh commands arrive.')
