#!/usr/bin/env python

import numpy as np

import actionlib
import rospy
import tf2_ros
import tf2_geometry_msgs

import alloy.ros
import alloy.math

from tbd_interaction_msgs.msg import (
    gazeAtAction,
    gazeAtGoal,
    gazeAtResult,
    moveToAction,
    moveToGoal,
    moveToResult
)


class PodiGazeMoveActuator():

    _tf_buffer: tf2_ros.Buffer

    _gaze_server: actionlib.SimpleActionServer
    _move_client: actionlib.SimpleActionClient

    def __init__(self):

        self._gaze_server = actionlib.SimpleActionServer(
            'actuator/gazeAt', gazeAtAction, self._gaze_cb, auto_start=False)
        self._move_client = actionlib.SimpleActionClient('actuator/moveTo', moveToAction)

        alloy.ros.ac_wait_for_server_wrapper(
            self._move_client.wait_for_server, f'move_client in {rospy.get_name()}')

        # start listening to TF
        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)

        self._gaze_server.start()

    def _gaze_cb(self, goal: gazeAtGoal) -> None:

        action_result = gazeAtResult()

        # depending on type
        if goal.gaze_type == goal.GAZE_AVERSION:

            action_result.code = gazeAtResult.RESULT_NOT_IMPLEMENTED
            self._gaze_server.set_succeeded(action_result)

        # all other scenarios are looking at something
        else:
            # figure out the target of the gaze
            target = goal.target

            # if the target isn't in `env` frame. transform it
            if target.header.frame_id != "env":
                attempts = 0
                while attempts < 10:
                    try:
                        transform = self._tf_buffer.lookup_transform(
                            'env', target.header.frame_id, target.header.stamp, rospy.Duration(0.5))
                        target = tf2_geometry_msgs.do_transform_point(target, transform)
                        break
                    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as err:
                    #     rospy.logwarn(f"Failed to transfrom from {target.header.frame_id} to env: {err}")
                    #     action_result.code = gazeAtResult.RESULT_CANNOT_FIND_TRANSFORMS
                    #     self._gaze_server.set_succeeded(action_result)
                    #     return
                    except (tf2_ros.ExtrapolationException, tf2_ros.LookupException, tf2_ros.ConnectivityException) as err:
                        rospy.logwarn(f"Failed to transfrom from {target.header.frame_id} to env:{err}")
                        attempts += 1
                        rospy.sleep(0.05)
                        continue
                if attempts >= 10:
                    rospy.logwarn(f"Failed to transfrom from {target.header.frame_id} to env: after 10 attempts of error")
                    action_result.code = gazeAtResult.RESULT_CANNOT_FIND_TRANSFORMS
                    self._gaze_server.set_succeeded(action_result)
                    return   

            # get the position of base_link
            podi_position = None
            attempts = 0
            while attempts < 10:
                try:
                    podi_position = self._tf_buffer.lookup_transform(
                        "env", "podi_base_link", target.header.stamp, rospy.Duration(0.25)).transform
                    break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
                    rospy.logwarn(f"Failed to transfrom from podi_base_link to env: {err}")
                    rospy.sleep(0.05)
                    attempts += 1
                    if attempts >= 10:
                        action_result.code = gazeAtResult.RESULT_CANNOT_FIND_TRANSFORMS
                        self._gaze_server.set_succeeded(action_result)
                        return

            # now we figure out the transformation needed
            target_point_np = alloy.ros.point_to_numpy(target.point)
            podi_mat_np = alloy.math.transformation_matrix_from_array(alloy.ros.transform_to_numpy(podi_position))

            direction_vector = np.zeros((3,))
            direction_vector[:2] = alloy.math.normalize(target_point_np[:2] - podi_mat_np[:2, 3])
            # use the direction matrix to complete transformation matrix
            trans_mat = podi_mat_np.copy()
            trans_mat[2, 3] = 0
            trans_mat[:3, 0] = direction_vector
            trans_mat[:3, 2] = np.array([0, 0, 1])
            trans_mat[:3, 1] = np.cross(trans_mat[:3, 2], trans_mat[:3, 0])

            # convert back to env frame
            # trans_mat = np.matmul(alloy.math.inverse_transformation_matrix(base_to_head_mat), trans_mat)

            # now we convert the transmat back
            target_pose = alloy.ros.numpy_to_pose(alloy.math.transformation_matrix_to_array(trans_mat))
            # send this movement to ros navigation to execute
            nav_goal = moveToGoal()
            nav_goal.desire_pose.pose = target_pose
            nav_goal.desire_pose.header.frame_id = "env"
            nav_goal.desire_pose.header.stamp = target.header.stamp
            nav_goal.header = alloy.ros.create_ros_header(rospy, "env")

            # send target and query
            self._move_client.send_goal(nav_goal)
            while not rospy.is_shutdown() and not self._gaze_server.is_preempt_requested():
                if self._move_client.wait_for_result(rospy.Duration(0.1)):
                    break

            # if shutting down
            if rospy.is_shutdown():
                return

            if self._gaze_server.is_preempt_requested():
                self._move_client.cancel_goal()
                action_result.code = gazeAtResult.RESULT_PREEMPTED
                self._gaze_server.set_preempted(action_result)
                return
            # else get the results
            move_result: moveToResult
            move_result = self._move_client.get_result()
            action_result.code = gazeAtResult.RESULT_SUCCESS if move_result.complete else gazeAtResult.RESULT_UNKNOWN_FAILURE
            self._gaze_server.set_succeeded(action_result)
