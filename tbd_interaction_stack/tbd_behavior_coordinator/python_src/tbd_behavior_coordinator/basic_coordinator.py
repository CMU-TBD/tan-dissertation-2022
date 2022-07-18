#!/usr/bin/env python3

import rospy
from tbd_interaction_msgs.msg import(
    speakToAction,
    speakToResult,
    gazeAtAction,
    gazeAtGoal
)
import actionlib
from actionlib_msgs.msg import GoalStatus

class BasicCoordinator():

    def initialize_behavior(self):

        num_service = 0
        # check gaze client
        self._gaze_server = None
        self._gaze_actuator_client = actionlib.SimpleActionClient("actuator/gazeAt", gazeAtAction)
        if self._gaze_actuator_client.wait_for_server(rospy.Duration(10)):
            num_service += 1
            # start the receivers
            self._gaze_server = actionlib.SimpleActionServer("gazeAt", gazeAtAction, execute_cb=self._gaze_at_cb, auto_start=False)
            self._gaze_server.start()

        # check speak client
        self._speak_server = None
        self._voice_actuator_client = actionlib.SimpleActionClient("actuator/speakTo", speakToAction)
        if self._voice_actuator_client.wait_for_server(rospy.Duration(10)):
            num_service += 1
            # start the receivers
            self._speak_server = actionlib.SimpleActionServer("speakTo", speakToAction, execute_cb=self._speak_to_cb, auto_start=False)
            self._speak_server.start()
        return num_service

    def __init__(self, expected_service: int = -1):

        service_num = self.initialize_behavior()
        if expected_service != -1 and service_num != expected_service:
            raise Exception(f"Coordinater expected {expected_service} services but only {service_num} activated")

        rospy.loginfo("Basic Coordinator Started.")


    def _speak_to_cb(self, goal):

        accompanied_gaze = False

        # redirect the goal to the actuators
        self._voice_actuator_client.send_goal(goal)

        # if user didn't specify gaze and there is target, try looking at the target
        if self._gaze_server is not None and not self._gaze_server.is_active() and goal.target.header.frame_id != "":
            rospy.logdebug("Having accompanying gaze...")
            gaze_goal = gazeAtGoal()
            gaze_goal.gaze_type = gazeAtGoal.GAZE_DIRECT
            # the gaze action expects a point but the speak action expects
            gaze_goal.target.point = goal.target.pose.position
            gaze_goal.target.header = goal.target.header
            self._gaze_actuator_client.send_goal(gaze_goal)
            accompanied_gaze = True
            # wait for 0.5 time to make it look natural
            rospy.sleep(0.5)
        
        # result object
        result = speakToResult()

        # check if it is preempted by the client
        while not self._voice_actuator_client.wait_for_result(rospy.Duration(secs=0.1)):
            if self._speak_server.is_new_goal_available() or self._speak_server.is_preempt_requested() or rospy.is_shutdown():
                # stop the current speech
                self._voice_actuator_client.cancel_goal()
                if accompanied_gaze:
                    self._gaze_actuator_client.cancel_goal()
                # end the current result
                result.success = 1
                self._speak_server.set_preempted(result)
                return

        # handle gaze.
        if accompanied_gaze:
            # As a choice, we just cancel it if it hasn't complete the action and not wait for it
            if self._gaze_actuator_client.get_state() == GoalStatus.ACTIVE:
                self._gaze_actuator_client.cancel_goal()

        # set the result on whether it successed.
        result = self._voice_actuator_client.get_result()
        self._speak_server.set_succeeded(result)

    def _gaze_at_cb(self, goal):
        # send the goal
        self._gaze_actuator_client.send_goal(goal)

        # check if we are going to pre-empted it
        preempted = False 
        while not self._gaze_actuator_client.wait_for_result(rospy.Duration(secs=0.1)):
            # check if pre-emptted or there is a new goal
            if self._gaze_server.is_new_goal_available() or self._gaze_server.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo("gaze behavior preempted or shutting down.")
                self._gaze_actuator_client.cancel_goal()
                preempted = True
                break
                
        result = self._gaze_actuator_client.get_result()

        if preempted:
            self._gaze_server.set_preempted(result)
        else:
            self._gaze_server.set_succeeded(result)
    

if __name__ == '__main__':
    rospy.init_node("basic_coordinator", log_level=rospy.DEBUG)
    behavior_num = rospy.get_param("~behavior_num", -1)
    coordinator = BasicCoordinator(behavior_num)
    rospy.spin()
