#!/usr/bin/env python3

import rospy
import actionlib

import alloy.ros
from tbd_interaction_msgs.msg import (
    speakToAction,
    speakToGoal
)


def main():
    # initialize node
    rospy.init_node('test_actuator')
    client = actionlib.SimpleActionClient('baxter/speakTo', speakToAction)
    client.wait_for_server()

    index = 0
    # until shutdown, take turns looking left and right
    while not rospy.is_shutdown():
        
        goal = speakToGoal()
        goal.utterance = "Looking to the " + ("left" if index == 0 else "right") + "."
        goal.target.header.frame_id = "env"
        goal.target.pose.position.x = 5
        goal.target.pose.position.y = 5 if index == 0 else -5
        goal.target.pose.position.z = 0.5
        goal.target.pose.orientation.w = 1

        client.send_goal_and_wait(goal)
        index = (index + 1)%2
        rospy.sleep(5)

if __name__ == '__main__':
    main()
