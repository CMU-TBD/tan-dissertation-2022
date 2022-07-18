#!/usr/bin/env python3

from tbd_ros_msgs.msg import (
    Agent
)
import rospy
import tf2_ros
from geometry_msgs.msg import (
    TransformStamped
)

import alloy.ros


class BaxterAgentNode():

    _tfBuffer: tf2_ros.Buffer
    _latest_msgs: Agent

    def __init__(self):

        self._pub = rospy.Publisher('/agent', Agent, queue_size=1)
        self._latest_msg = Agent()
        self._latest_msg.agent_id = 'baxter'
        self._latest_msg.agent_type_unique = 'baxter'

        # get the location of the world
        self._tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tfBuffer)

    def get_transform(self, target_frame: str, original_frame: str) -> TransformStamped:
        try:
            trans = self._tfBuffer.lookup_transform(
                original_frame, target_frame, rospy.Time())
            # TODO convert the head to the attention array
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
            rospy.logwarn(
                f"Unable to find transform from {original_frame} to {target_frame}: {err}")
            return None
        return trans

    def spin(self, hz=10):

        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():

            # Baxter is fixed in place, so we only need the base location
            trans = self.get_transform('base', 'env')
            if trans != None:
                self._latest_msg.center_pose.pose = alloy.ros.transform_to_pose(
                    trans.transform)
                self._latest_msg.center_pose.header = trans.header

            # get the location of baxter's screen
            head_trans = self.get_transform('screen', 'env')
            if head_trans != None:
                self._latest_msg.attention.pose = alloy.ros.transform_to_pose(
                    head_trans.transform)
                self._latest_msg.attention.header = head_trans.header

            self._pub.publish(self._latest_msg)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("baxter_agent_node")
    agent = BaxterAgentNode()
    agent.spin()
