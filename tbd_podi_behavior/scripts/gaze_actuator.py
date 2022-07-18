#!/usr/bin/env python3

import rospy
from tbd_podi_behavior.gaze_screen_controller import PodiGazeActuator
from tbd_podi_behavior.gaze_move_controller import PodiGazeMoveActuator

if __name__ == "__main__":
    rospy.init_node("gaze_actuator")
    # controller = PodiGazeActuator()
    controller = PodiGazeMoveActuator()
    rospy.spin()
