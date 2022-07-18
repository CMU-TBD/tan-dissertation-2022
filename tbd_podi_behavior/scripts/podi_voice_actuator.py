#!/usr/bin/env python3
import os.path as path
import rospy
from std_msgs.msg import Bool
from sound_play.libsoundplay import SoundClient
from tbd_polly_speech import PollySpeech
from tbd_audio_msgs.msg import Utterance
from tbd_interaction_msgs.msg import (
    speakToGoal,
    speakToAction,
    speakToResult,
)
import actionlib
import alloy.ros


class PodiVoiceActuator():

    _speak_server: actionlib.SimpleActionServer
    _speak_feedback_pub: rospy.Publisher
    _sound_play: SoundClient
    _speak_signal_msg: Bool

    def __init__(self):

        self._pollyspeech = PollySpeech()
        self._speak_server = actionlib.SimpleActionServer(
            "actuator/speakTo", speakToAction, self.speak_to_cb, auto_start=False)
        self._speak_feedback_pub = rospy.Publisher("action_feedback/voice", Utterance, queue_size=1)
        self._speak_signal_pub = rospy.Publisher("speaking_signal", Bool, queue_size=1)
        self._speak_signal_msg = Bool(data=False)
        self._sound_play = SoundClient()
        self._speak_server.start()

    def _play_sound(self, sound):
        wave_path = path.join(alloy.ros.get_res_path("tbd_podi_behavior"), "audio", f"{sound}.wav")
        self._sound_play.playWave(wave_path)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._speak_signal_pub.publish(self._speak_signal_msg)
            rate.sleep()


    def speak_to_cb(self, goal: speakToGoal):
        # result object
        result = speakToResult()
        # get text
        text = goal.utterance
        if text.lower().startswith("sound:"):
            self._speak_signal_msg.data = True
            self._play_sound(text[6:])
            self._speak_signal_msg.data = False
            result.success = 0
            self._speak_server.set_succeeded(result)
            return
        # create feedback object
        feedback_msg = Utterance(text=text)
        feedback_msg.header.stamp = rospy.get_rostime()
        # start the speech
        self._speak_signal_msg.data = True
        self._pollyspeech.speak(text, block=False)

        # check if it is preempted by the client
        while not self._pollyspeech.wait(rospy.Duration(secs=0.1)):
            if self._speak_server.is_new_goal_available() or self._speak_server.is_preempt_requested():
                # stop the current speech
                self._pollyspeech.stop()
                self._speak_signal_msg.data = False
                # end the current result
                result.success = 1
                self._speak_server.set_preempted(result)
                return

        # if successful also send out the information
        feedback_msg.end_time = rospy.get_rostime()
        self._speak_feedback_pub.publish(feedback_msg)
        self._speak_signal_msg.data = False
        # set result
        result.success = 0
        self._speak_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node("polly_voice_actuator")
    polly_voice_actuator = PodiVoiceActuator()
    polly_voice_actuator.spin()
