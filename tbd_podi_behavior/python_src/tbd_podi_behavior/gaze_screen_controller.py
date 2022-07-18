#!/usr/bin/env python

import rospy
from tbd_interaction_msgs.msg import (
    gazeAtAction,
    gazeAtGoal,
    gazeAtResult
)

from tbd_ros_msgs.msg import (
    faceAnimationAction,
    faceAnimationGoal,
    FaceAnimationObject
)

from geometry_msgs.msg import (
    TransformStamped
)

import tf2_ros
import actionlib
import alloy.ros
import numpy as np
from alloy.spatial.primitives import Ray, Polygon
import alloy.math


class PodiGazeActuator():

    _tfBuffer: tf2_ros.Buffer
    _tfListener: tf2_ros.TransformListener
    _screen_width: int
    _screen_height: int

    _default_left_eye_loc = [0.25, 0.35]
    _default_right_eye_loc = [0.75, 0.35]

    def __init__(self):

        self._face_client = actionlib.SimpleActionClient(
            'animation', faceAnimationAction)
        alloy.ros.ac_wait_for_server_wrapper(
            self._face_client.wait_for_server, 'face_client_in_podi_gaze')

        self._gaze_server = actionlib.SimpleActionServer(
            'actuator/gazeAt', gazeAtAction, self._gaze_cb, auto_start=False)
        self._gaze_server.start()

        self._screen_height = 0.15  # in meters
        self._screen_width = 0.3  # in meters

        # Listen to TF to get information about robot's current location
        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)

    def get_transform(self, original_frame: str, target_frame: str) -> TransformStamped:
        try:
            trans = self._tfBuffer.lookup_transform(
                original_frame, target_frame, rospy.Time())
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
            rospy.logwarn(
                f"Unable to find transform from {original_frame} to {target_frame}: {err}")
            return None
        return trans

    @staticmethod
    def _calculate_screen_location(target_in_head_frame: np.array, screen_width: int, screen_height: int):

        # calculate a point that is about 0.05m behind the head.
        # The focal point in th head frame
        focal_point = np.array([-0.05, 0, 0])

        # calculate a ray from the focal_point to the target
        ray_dir = target_in_head_frame[0:3] - focal_point
        ray_dir = ray_dir/np.linalg.norm(ray_dir)

        # if the ray is behind us, we can't do anything
        if ray_dir[0] <= 0:
            return (gazeAtResult.RESULT_CANNOT_FIND_TRANSFORMS, None)

        # see where the ray intersect the screen
        verticies = np.array([
            [0, screen_width/2.0, screen_height/2.0],
            [0, -screen_width/2.0, screen_height/2.0],
            [0, screen_width/2.0, -screen_height/2.0],
            [0, -screen_width/2.0, -screen_height/2.0]
        ])
        screen_polygon = Polygon(verticies)

        # calculate where it will intersect the face screen.
        ray = Ray(focal_point[0:3], ray_dir)
        result = screen_polygon.intersect_with_ray(ray)
        if result is None:
            # if thre ray doesn't intersect with the face screen.
            # return an error code.
            return (gazeAtResult.RESULT_TARGET_OUT_OF_VIEW_RANGE, None)

        # calculate the location on the screen
        x_pos = 1 - ((screen_width/2.0 + result[1])/screen_width)
        y_pos = 1 - ((screen_height/2.0 + result[2])/screen_height)

        return (0, [x_pos, y_pos])

    def _gaze_cb(self, goal: gazeAtGoal) -> None:

        action_result = gazeAtResult()

        # depending on type
        if (goal.gaze_type == goal.GAZE_DEFAULT):

            # set it to the default
            animation_goal = faceAnimationGoal()
            gaze_duration = 0.5
            animation_goal.objects = [
                FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="left_eye.center",
                                    target=self._default_left_eye_loc, duration=gaze_duration),
                FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="right_eye.center",
                                    target=self._default_right_eye_loc, duration=gaze_duration)
            ]
            # gaze at other object
            self._face_client.send_goal_and_wait(animation_goal)

            action_result.code = gazeAtResult.RESULT_SUCCESS
            self._gaze_server.set_succeeded(action_result)

        elif (goal.gaze_type == goal.GAZE_AVERSION):

            action_result.code = gazeAtResult.RESULT_NOT_IMPLEMENTED
            self._gaze_server.set_succeeded(action_result)

        # directly looking at something
        elif (goal.gaze_type == goal.GAZE_DIRECT):

            # figure out the target of the gaze
            target = goal.target

            # convert the world pose back to head
            world_to_head_trans = self.get_transform(
                'podi/head', target.header.frame_id)
            if world_to_head_trans is None:
                # cannot figure out the transformation of the head
                # we do nothing
                action_result.code = gazeAtResult.RESULT_CANNOT_FIND_TRANSFORMS
                self._gaze_server.set_succeeded(action_result)
                return

            # transfom the point from world frame to head frame
            world_to_head_trans_np = alloy.ros.transform_to_numpy(
                world_to_head_trans.transform)
            world_to_head_trans_mat = alloy.math.transformation_matrix_from_array(
                world_to_head_trans_np)
            target_np = alloy.ros.point_to_numpy(target.point)
            target_in_head_frame = np.matmul(world_to_head_trans_mat, np.append(
                target_np, 1).reshape((4, 1))).reshape((4,))

            (result_code, location) = self._calculate_screen_location(
                target_in_head_frame, self._screen_width, self._screen_height)

            if result_code != 0:
                action_result.code = result_code
                self._gaze_server.set_succeeded(action_result)

            eye_center = [location[0] * 0.8, location[1] * 0.8]

            # get eye center
            left_eye_center = np.array(eye_center)
            left_eye_center[0] -= 0.20
            right_eye_center = np.array(eye_center)
            right_eye_center[0] += 0.20

            gaze_duration = 0.5

            animation_goal = faceAnimationGoal()
            animation_goal.objects = [
                FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="left_eye.center",
                                    target=left_eye_center, duration=gaze_duration),
                FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="right_eye.center",
                                    target=right_eye_center, duration=gaze_duration)
            ]
            # gaze at other object
            self._face_client.send_goal_and_wait(animation_goal)

            action_result.code = gazeAtResult.RESULT_SUCCESS
            self._gaze_server.set_succeeded(action_result)
