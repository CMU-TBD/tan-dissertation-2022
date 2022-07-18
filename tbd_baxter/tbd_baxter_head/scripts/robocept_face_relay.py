#!/usr/bin/env python3

import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import (
    Image
)


def main():
    #initialize node
    rospy.init_node('baxter_robocept_face_relay')

    pub = rospy.Publisher("face_out", Image, queue_size=1)
    
    bridge = cv_bridge.CvBridge()
    def face_in_cb(msg: Image):
        cv2_img = bridge.imgmsg_to_cv2(msg)
        if msg.width == msg.height == 600:
            # resize to the size we want (600x1024) for baxter
            pad_size = (1024 - 600)//2
            padded_cv2_img = cv2.copyMakeBorder(cv2_img,0,0,pad_size, pad_size, cv2.BORDER_CONSTANT)
            padded_msg = bridge.cv2_to_imgmsg(padded_cv2_img, encoding='bgr8')
            pub.publish(padded_msg)
        elif msg.width == msg.height == 1024:
            crop_size = (1024-600)//2
            crop_img = cv2_img[crop_size:1024-crop_size, 0:1024].copy()
            crop_msg = bridge.cv2_to_imgmsg(crop_img, encoding='bgr8')
            pub.publish(crop_msg)
        else:
            rospy.logerr(f"Receive Image of unsupported size:{msg.width}x{msg.height}")


    rospy.Subscriber("face_in", Image, face_in_cb, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
