#!/usr/bin/env python3

import rospy
from tbd_baxter_head.HeadController import HeadController

if __name__ == '__main__':
    rospy.init_node("test_head_controller")

    controller = HeadController()
    # move the head to the left
    controller.move_head(-1, rospy.Duration(1))
    controller.move_head(1, rospy.Duration(1))