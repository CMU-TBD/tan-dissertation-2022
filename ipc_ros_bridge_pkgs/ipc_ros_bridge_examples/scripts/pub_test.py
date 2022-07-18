#!/usr/bin/env python

import rospy
from std_msgs.msg import(
    String,
    Float64
)

if __name__ == "__main__":
    rospy.init_node("test")

    pub = rospy.Publisher('t', String, queue_size=1)
    pub2 = rospy.Publisher('f', Float64, queue_size=1)

    msg = String()
    msg2 = Float64()

    msg.data = "Hello"
    
    counter = 0.1

    while not rospy.is_shutdown():
        pub.publish(msg)
        msg2.data = counter
        counter += 0.1
        pub2.publish(msg2)
        rospy.sleep(1)
