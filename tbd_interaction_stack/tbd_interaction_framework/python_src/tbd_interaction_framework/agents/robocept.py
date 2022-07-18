"""
Copyright - Transporation, Bots, and Disability Lab - Carnegie Mellon University
Released under MIT License

agents/robocept.py - Base class for a robocept agent.
"""

import alloy.ros
import alloy.math
from tbd_interaction_framework.entities.agent import Agent
from tbd_ros_msgs.msg import Agent as Agent_msgs


class RoboceptAgentState():
    """ Parser for robocept agent message to robocept object.
    """

    @staticmethod
    def from_agent_msg(previous: Agent, msg: Agent_msgs) -> Agent:
        """Updates a previous Agent object with information from the latest Agent Msg.

        Args:
            previous (Agent): Previous Agent Object.
            msg (Agent_msgs): ROS Message about the Agent's new state.

        Returns:
            Agent: Updated Agent.
        """

        agent = previous
        if msg.center_pose.header.frame_id == 'env':
            center_pose_np = alloy.ros.pose_to_numpy(msg.center_pose.pose)
            agent.base = alloy.math.transformation_matrix_from_array(center_pose_np)

        if msg.attention.header.frame_id == 'env':
            attention_np = alloy.ros.pose_to_numpy(msg.attention.pose)
            attention_trans_mat = alloy.math.transformation_matrix_from_array(attention_np)
            agent.attention_origin = attention_trans_mat[0:3, 3]  # The position of the head
            agent.attention_ray = attention_trans_mat[:3, 0]  # The X-axis should be pointing forward.

        return agent
