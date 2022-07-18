#!/usr/bin/env python3

import rospy
from tbd_polly_speech import PollySpeech
from tbd_audio_msgs.msg import Utterance
from tbd_interaction_msgs.msg import(
    speakToGoal,
    speakToAction,
    speakToResult,
)
import actionlib


class VoiceActuator():

    _speak_server: actionlib.SimpleActionServer
    _speak_feedback_pub: rospy.Publisher
    def __init__(self):

        self._pollyspeech = PollySpeech()
        self._speak_server = actionlib.SimpleActionServer(
            "actuator/speakTo", speakToAction, self.speak_to_cb, auto_start=False)
        self._speak_feedback_pub = rospy.Publisher("action_feedback/voice", Utterance, queue_size=1)

        self._speak_server.start()

    def speak_to_cb(self, goal: speakToGoal):
        # get text
        text = goal.utterance
        # create feedback object 
        feedback_msg = Utterance(text=text)
        feedback_msg.header.stamp = rospy.get_rostime()
        # start the speech
        self._pollyspeech.speak(text, block=False)
        # result object
        result = speakToResult()
        # check if it is preempted by the client
        while not self._pollyspeech.wait(rospy.Duration(secs=0.1)):
            if self._speak_server.is_new_goal_available() or self._speak_server.is_preempt_requested():
                # stop the current speech
                self._pollyspeech.stop()
                # end the current result
                result.success = 1
                self._speak_server.set_preempted(result)
                return

        # if successful also send out the information
        feedback_msg.end_time = rospy.get_rostime()
        self._speak_feedback_pub.publish(feedback_msg)

        # set result
        result.success = 0
        self._speak_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node("polly_voice_actuator")
    polly_voice_actuator = VoiceActuator()
    rospy.spin()
