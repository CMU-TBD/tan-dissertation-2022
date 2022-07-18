#!/usr/bin/python

import rospy
from tbd_audio_common.sound_maker import SoundMaker


if __name__ == "__main__":
    rospy.init_node('sound_maker_test')
    sm = SoundMaker()
    sm.play_beep()