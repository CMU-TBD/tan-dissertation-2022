"""
Copyright - Transporation, Bots, and Disability Lab - Carnegie Mellon University
Released under MIT License

agents/computer.py - Base class for a computer agent.
"""

from tbd_interaction_framework.entities.agent import Agent
from tbd_ros_msgs.msg import Agent as Agent_msgs


class ComputerAgentState():
    """ Parser for robocept agent message to robocept object.
    """

    @staticmethod
    def from_agent_msg(previous: Agent, _msg: Agent_msgs):
        """Updates a previous Agent object with information from the latest Agent Msg.

        Args:
            previous (Agent): Previous Agent Object.
            _msg (Agent_msgs): ROS Message about the Agent's new state.

        Returns:
            Agent: Updated Agent.
        """
        agent = previous
        return agent
