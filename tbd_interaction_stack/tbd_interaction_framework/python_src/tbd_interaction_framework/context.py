"""
Copyright - Transporation, Bots, and Disability Lab - Carnegie Mellon University
Released under MIT License

context.py - Encodes context of the scene.
"""

import typing

import rospy

from tbd_audio_msgs.msg import Utterance
from tbd_ros_msgs.msg import HumanBodyArray

from .entities.human import Human
from .entities.agent import Agent
from .interaction import Interaction


class Context():
    """ Object describing the scene at the current time.
    """

    last_update_time: float                 # last update time
    utterances: list                        # List of utterances
    humans: typing.Mapping[str, Human]      # List of all previous & active humans
    active_humans: list                     # List of all active humans
    agents: typing.Mapping[str, Agent]      # List of agents
    main_agent: Agent                       # main interacting agent
    main_agent_id: str                      # main interacting agent ID
    interaction: Interaction

    def __init__(self, main_id):

        # initialize local variables
        self.utterances = []
        self.humans = {}
        self.active_humans = []
        self.agents = {}
        self.main_agent_id = main_id
        self.main_agent = None
        self.last_update_time = 0
        # create high level interaction obj
        self.interaction = Interaction()

    def update_humans(self, humans_msg: HumanBodyArray) -> None:
        """ Update the context with new observed humans.

        Args:
            humans_msg (HumanBodyArray): humans we observe at the current time.
        """

        self.last_update_time = rospy.get_time()
        # add the humans to the list
        self.active_humans.clear()  # clear out the active humans
        for body in humans_msg.bodies:
            human_obj = Human(body)
            self.humans[body.body_id] = human_obj
            self.active_humans.append(human_obj)
        # update interaction
        self.interaction.update(self.main_agent, self.agents, self.active_humans)

    def update_utterances(self, utterance_msg: Utterance) -> None:
        """ Update utterances that you hear.

        Args:
            utterance_msg (Utterance): Utterance ROS Message.
        """
        self.last_update_time = rospy.get_time()
        # add the latest utterance
        self.utterances.append(utterance_msg)

    def update_agents(self, agent: Agent) -> None:
        """ Update the context with an agents newest state.

        Args:
            agent (Agent): Agent to be updated.
        """
        self.last_update_time = rospy.get_time()
        self.agents[agent.id] = agent
        if agent.id == self.main_agent_id:
            self.main_agent = agent
        # update interaction
        self.interaction.update(self.main_agent, self.agents, self.active_humans)
