#!/usr/bin/env python3

import numpy as np
import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs

from tbd_interaction_msgs.msg import (
    moveToAction,
    moveToGoal,
    moveToResult
)
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal
)
from geometry_msgs.msg import (
    Twist,
    PoseStamped
)

import alloy.ros
import alloy.math


class PodiMoveActuator():

    _tf_buffer: tf2_ros.Buffer

    def __init__(self):

        # the backend operator
        self._move_actuator_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        rospy.sleep(0.5)
        if not self._move_actuator_client.wait_for_server(rospy.Duration(120)):
            raise Exception("unable to connect to move_base server")

        self._move_server = actionlib.SimpleActionServer(
            "actuator/moveTo", moveToAction, execute_cb=self._move_to_cb, auto_start=False)
        self._move_server.start()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._cmd_vel_pub = rospy.Publisher("nav_cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Podi Move Actuator Started.")

    def get_podi_position_in_world(self, world_frame_id: str = "env"):

        try:
            transform_stamped = self._tf_buffer.lookup_transform(world_frame_id, "podi_base_link", rospy.Time())
            return alloy.ros.to_pose_stamped(transform_stamped)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
            rospy.logerr(f"Unable to find transform from {world_frame_id} to podi_base_link: {err}")
            return None

    def _move_to_cb(self, goal: moveToGoal):

        result = moveToResult()

        # try to get the current position of the robot
        try:
            curr_pose = self._tf_buffer.lookup_transform(
                "env",
                "podi_base_link",
                rospy.Time()).transform

            goal_pose = goal.desire_pose.pose
            goal_stamped_pose = goal.desire_pose

            # if the pose isn't that different from the goal
            if goal.desire_pose.header.frame_id != "env":
                # transform the goal
                attempts = 0
                while attempts <= 5:
                    try:
                        transform = self._tf_buffer.lookup_transform("env", goal.desire_pose.header.frame_id, rospy.Time())
                        goal_stamped_pose = tf2_geometry_msgs.do_transform_pose(goal_stamped_pose, transform)
                        goal_pose = goal_stamped_pose.pose
                        break
                    except (tf2_ros.ExtrapolationException, tf2_ros.LookupException, tf2_ros.ConnectivityException) as err:
                        if attempts == 5:
                            rospy.logwarn(f"Failed to transfrom from {goal.desire_pose.header.frame_id} to env: after 5 attempts of error: {err}")
                            result.complete = False
                            self._move_server.set_succeeded(result)
                            return
                        rospy.logwarn(f"Failed to transfrom from {goal.desire_pose.header.frame_id} to env: {err}: RETRYINGGGG")
                        attempts += 1
                        rospy.sleep(0.05)
                        continue
                    
            # if they are really close, we use manual turning and skip ROS Navigation
            goal_np = alloy.ros.pose_to_numpy(goal_pose)
            current_np = alloy.ros.transform_to_numpy(curr_pose)

            trans_diff = np.sqrt(np.sum((goal_np[:3] - current_np[:3]) * (goal_np[:3] - current_np[:3])))
            if trans_diff < 0.2:
                # a simple PD controller
                twist = Twist()
                hz = 50
                last_error = 0
                KP = 3
                KD = 0
                rate = rospy.Rate(hz)
                while not rospy.is_shutdown() and not self._move_server.is_preempt_requested():

                    # get the latest position of the robot.
                    curr_transform = self._tf_buffer.lookup_transform("podi_base_link", "env", rospy.Time())
                    # find the location of the pose in the `podi_base_link` frame. 
                    posestamped_in_podi_frame = tf2_geometry_msgs.do_transform_pose(goal_stamped_pose, curr_transform)
                    goal_rot_mat = alloy.math.transformation_matrix_from_array(alloy.ros.pose_to_numpy(posestamped_in_podi_frame.pose))
                    # find the rotation between X-axis of Podi and the position
                    err_yaw = alloy.math.find_rotation([1,0], goal_rot_mat[:2,0])
                    # stop if we are close to target
                    if np.abs(err_yaw) < 0.05:
                        break
                    
                    # do PD controller
                    d_err = err_yaw - last_error
                    pwd = err_yaw * KP + d_err/(1/hz) * KD

                    # update for next loop
                    last_error = err_yaw

                    # clip the maximum speed
                    twist.angular.z = np.min([0.9, np.abs(pwd)])
                    # set the direction to be the opposite.
                    twist.angular.z *= np.sign(err_yaw)
                    # publish and sleep
                    self._cmd_vel_pub.publish(twist)
                    rate.sleep()

                # we stopped for some reason. Let's publish empty twist first
                twist = Twist()
                for _ in range(0, int(hz/2)):
                    self._cmd_vel_pub.publish(twist)
                    rate.sleep()

                # get podi's final position
                final_pose_stamped = self.get_podi_position_in_world()
                if final_pose_stamped is not None:
                    result.final_pose = final_pose_stamped
                result.complete = (not rospy.is_shutdown() and not self._move_server.is_preempt_requested())
                self._move_server.set_succeeded(result)
                return

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            # we cannot find the position, so let's just send it to ROS Navigation to handle it
            rospy.logerr(ex)

        # convert the moveTo Intent into movebase messge
        rosnav_goal = MoveBaseGoal()
        rosnav_goal.target_pose = goal.desire_pose
        # wait for the response
        self._move_actuator_client.send_goal(rosnav_goal)
        while not rospy.is_shutdown() and not self._move_actuator_client.wait_for_result(rospy.Duration(0.5)):
            if self._move_server.is_preempt_requested():
                self._move_actuator_client.cancel_all_goals()
                result.complete = False
                # try to find the current pose
                curr_pose_stamped = self.get_podi_position_in_world()
                result.final_pose = PoseStamped() if curr_pose_stamped is None else curr_pose_stamped
                self._move_server.set_succeeded(result)
                return
        result.complete = True
        curr_pose_stamped = self.get_podi_position_in_world()
        result.final_pose = PoseStamped() if curr_pose_stamped is None else curr_pose_stamped
        self._move_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node("podi_move_actuator")
    # There's a bug where in simulator, rospy takes awhile to get the correct time.
    # this make sure it at least heard one of the time msg.
    rospy.sleep(0.1)
    actuator = PodiMoveActuator()
    rospy.spin()
