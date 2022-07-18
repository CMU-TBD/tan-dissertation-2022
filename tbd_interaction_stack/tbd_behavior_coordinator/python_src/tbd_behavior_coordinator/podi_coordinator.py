#!/usr/bin/env python3

import rospy
import actionlib

from tbd_behavior_coordinator.basic_coordinator import BasicCoordinator

from tbd_interaction_msgs.msg import(
    moveToAction,
    moveToGoal,
)


class PodiCoordinator(BasicCoordinator):

    def initialize_behavior(self):

        service_num = 0
        self._move_actuator_client = actionlib.SimpleActionClient(
            "actuator/moveTo", moveToAction)
        if self._move_actuator_client.wait_for_server(rospy.Duration(30)):
            self._move_server = actionlib.SimpleActionServer(
                "moveTo", moveToAction, execute_cb=self._move_to_cb, auto_start=False)
            self._move_server.start()
            service_num += 1
        else:
            rospy.logwarn("Podi unable to connect to moveTo actuator server")

        service_num += super().initialize_behavior()
        return service_num

    def __init__(self, expected_num = -1):

        service_num = self.initialize_behavior()
        if expected_num != -1 and service_num != expected_num:
            raise Exception(f"Coordinater expected {expected_num} services but only {service_num} activated")

        rospy.loginfo("Podi Coordinator Started.")

    def _move_to_cb(self, goal: moveToGoal):

        # redirect the goal
        self._move_actuator_client.send_goal_and_wait(goal)
        result = self._move_actuator_client.get_result()
        self._move_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node("podi_coordinator")
    behavior_num = rospy.get_param("~behavior_num", -1)
    coordinator = PodiCoordinator(behavior_num)
    rospy.spin()
