#!/usr/bin/env python3

import numpy as np
import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs

from tbd_interaction_msgs.msg import (
    gazeAtAction,
    gazeAtResult
)
from tbd_baxter_head.HeadController import HeadController


class HeadActuator(object):

    def __init__(self):
        self._head_controller = HeadController()

        # the frame we are working in
        self._head_frame_id = "base"  # use sonar ring because its fixed

        # listen to TF to transform different frames
        self._tf_Buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_Buffer)

        self._gaze_at_result = gazeAtResult()
        self._as = actionlib.SimpleActionServer(
            "actuator/gazeAt", gazeAtAction, execute_cb=self._gaze_at_cb, auto_start=False)
        self._as.start()

    def _gaze_at_cb(self, goal):
        # try to figure out the target
        target_point = goal.target.point
        if goal.target.header.frame_id != self._head_frame_id:
            try:
                # get the transform
                transform = self._tf_Buffer.lookup_transform(
                    self._head_frame_id, goal.target.header.frame_id, goal.target.header.stamp, rospy.Duration(secs=5))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                rospy.logerr(ex)
                # unable to get a transform.
                self._gaze_at_result.code = gazeAtResult.RESULT_CANNOT_FIND_TRANSFORMS
                self._as.set_succeeded(self._gaze_at_result)
                return

            # transform the point from the source to head frame.'
            target_point = (tf2_geometry_msgs.do_transform_point(
                goal.target, transform)).point

        target_point.z -= 0.686  # Do the correction up to the head position
        target_point.x -= 0.06
        # calculate the desire yaw and pitch
        desire_yaw = np.arctan2(target_point.y, target_point.x)
        rospy.logdebug(f"moving head to {desire_yaw}")
        desire_pitch = np.arctan2(target_point.z, target_point.x)
        # set the head positon
        if self._head_controller.move_head(desire_yaw, rospy.Duration(1)):
            self._gaze_at_result.code = gazeAtResult.RESULT_SUCCESS
        else:
            self._gaze_at_result.code = gazeAtResult.RESULT_CANNOT_FIND_TRANSFORMS
        self._as.set_succeeded(self._gaze_at_result)
        rospy.logdebug("head_behavior_controller done")


def main():
    # initialize node
    rospy.init_node('baxter_head_behavior_controller', log_level=rospy.DEBUG)
    # start controller
    server = HeadActuator()
    rospy.loginfo("Starting Baxter Behavior Head Controller")
    # spin until quit
    rospy.spin()


if __name__ == '__main__':
    main()
