#!/usr/bin/python3

from tbd_baxter_head.HeadController import HeadController

import rospy

if __name__ == "__main__":
    rospy.init_node('move_head_demo')
    hm = HeadController()
    hm.move_head(1, rospy.Duration(10), wait=True)
    rospy.sleep(1)
    hm.move_head(-1, rospy.Duration(1), wait=True)
    rospy.sleep(1)
    hm.move_head(0, rospy.Duration(0), wait=True)
    rospy.sleep(1)
    hm.move_head(1, rospy.Duration(5), wait=False)
    rospy.sleep(1)
    hm.move_head(-1, rospy.Duration(1), wait=True)
