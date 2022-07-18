#!/usr/bin/env python3

import rospy
from tbd_interaction_msgs.msg import (
    gazeAtAction,
    gazeAtGoal
)

import actionlib
import alloy.ros

if __name__ == "__main__":
    rospy.init_node("test_gaze")

    # create action client
    ac = actionlib.SimpleActionClient("podi/gazeAt", gazeAtAction)
    if not alloy.ros.ac_wait_for_server_wrapper(ac.wait_for_server, "test_gaze_gaze_controller"):
        raise ValueError("Cannot connect to gaze server")

    # create some kind of gaze target
    goal = gazeAtGoal()
    goal.target.header = alloy.ros.create_ros_header(rospy, 'env')
    goal.target.point.y = 0
    goal.target.point.x = 0

    # send the goal target
    ac.send_goal_and_wait(goal)
