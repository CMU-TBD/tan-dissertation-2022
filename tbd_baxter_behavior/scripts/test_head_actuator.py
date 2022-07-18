#!/usr/bin/env python3

import rospy
import actionlib

import alloy.ros
from tbd_interaction_msgs.msg import (
    gazeAtAction,
    gazeAtGoal
)


def main():
    # initialize node
    rospy.init_node('test_head_actuator')
    client = actionlib.SimpleActionClient('baxter/actuator/gazeAt', gazeAtAction)
    client.wait_for_server()

    for i in range(0, 20):

        goal = gazeAtGoal()
        goal.target.header = alloy.ros.create_ros_header(rospy, "base")
        goal.target.point.x = 5
        goal.target.point.y = (i - 10)
        goal.target.point.z = 0
        print(goal.target.point)
        client.send_goal(goal)
        client.wait_for_result()

        print(client.get_result())
        #rospy.sleep(0.1)
        if rospy.is_shutdown():
            break


if __name__ == '__main__':
    main()
