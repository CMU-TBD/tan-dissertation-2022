#!/usr/bin/env python

import rospy
import alloy.ros
import os 
import wave
import actionlib

from tbd_ros_msgs.msg import (
    playAudioAction,
    playAudioGoal
)

class SoundMaker():

    def __init__(self):
        self._tbd_audio_client = actionlib.SimpleActionClient("playAudio", playAudioAction)
        self._tbd_imported_playAudioGoal = playAudioGoal
        self._tbd_audio_client.wait_for_server()

        self._res_dir = alloy.ros.get_res_path('tbd_audio_common')

    def play_beep(self, block=True):

        #get the 
        waveFile = wave.open(os.path.join(self._res_dir,'beep.wav'))
        num_of_frames = waveFile.getnframes() * waveFile.getsampwidth()
        #generate goal
        goal = playAudioGoal()
        goal.soundFile = waveFile.readframes(num_of_frames)
        goal.rate = int(waveFile.getframerate())
        goal.size = num_of_frames
        #send to the goal server
        if block:
            self._tbd_audio_client.send_goal_and_wait(goal)
        else:
            self._tbd_audio_client.send_goal(goal)
    
    def wait(self, duration=None):
        """
        Wait for the sound to finish. Note, sometimes the last few seconds of the speech will still be playing when it ends
        
        Parameters
        ----------
        duration : rospy.Duration
            Ros's implementation of Duration

        """
        if self._tbd_audio_client.gh:
            if duration is not None:
                result = self._tbd_audio_client.wait_for_result(duration)
            else:
                result = self._tbd_audio_client.wait_for_result()
