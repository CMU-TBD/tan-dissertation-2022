"""
Copyright - Transporation, Bots, and Disability Lab - Carnegie Mellon University
Released under MIT License

merger.py - ROS component that listen to varies messages and update the context.
"""

import copy
import threading
import importlib

import rospy
import alloy.ros

from tbd_audio_msgs.msg import(
    Utterance
)
from geometry_msgs.msg import (
    Point
)
from tbd_ros_msgs.msg import (
    HumanBodyArray,
    Agent,
    InteractionSpaceArray,
    InteractionSpace
)

from .entities.agent import Agent as AgentObj
from .context import Context


class ContextMerger():
    """
    This class keeps track of the context of the interactions and updates it as new
    information flows into the objects
    """

    _context: Context
    _context_lock: threading.RLock
    _info: dict
    _update_timer: rospy.Timer
    _publish_timer: rospy.Timer

    def __init__(self, info=None, main_agent_id=""):

        # TODO merge context and info
        self._context = Context(main_id=main_agent_id)
        self._context_lock = threading.RLock()
        self._info = info if info is not None else {}
        self._update_timer = None

        """ ---- Audio & Languages ---- """
        # subscribe to the utterance packages if exist
        rospy.Subscriber('utterance', Utterance, self._utterance_cb)

        """ ---- People & Objects ----- """
        # subscribe to the people
        rospy.Subscriber('humans', HumanBodyArray, self._human_cb)

        """ ---- Get Information about Agents ---- """
        # agent info
        rospy.Subscriber('agent', Agent, self._agent_cb)

        """ --- Publishers for external stuff --- """
        self._interaction_pub = rospy.Publisher("interaction_space", InteractionSpaceArray, queue_size=1)

    def start_listening(self):
        """Initialize the merger to start publishing or do periodical queries.
        """
        self._update_timer = rospy.Timer(rospy.Duration(secs=1/50), self._interval_update)
        self._publish_timer = rospy.Timer(rospy.Duration(secs=1/5), self._publish)

    def _utterance_cb(self, msg):
        with self._context_lock:
            self._context.update_utterances(msg)

    def _human_cb(self, msg):
        with self._context_lock:
            # update humans
            self._context.update_humans(msg)

    def _agent_cb(self, msg):
        with self._context_lock:
            # dependening on the type of agent, load the correct types
            if 'agent_types' in self._info:
                # get the type of agent and split it into module and class
                agent_type = self._info['agent_types'][msg.agent_type_unique].split('.')
                mod = importlib.import_module(f".agents.{agent_type[0]}", package=__package__)
                class_obj = getattr(mod, agent_type[1])  # get the class object
                # get the static method "from_agent_msg" from the class
                converter = getattr(class_obj, "from_agent_msg")
                try:
                    previous_agent = self._context.agents[msg.agent_id]
                except KeyError:
                    previous_agent = AgentObj(msg.agent_id)
                result = converter(previous_agent, msg)  # convert the message
                self._context.update_agents(result)

    def _interval_update(self, _event):
        # get newest robot position
        # self._context.update_robots(self._robot_info.get_robot_state())
        pass

    def _publish(self, _event):
        # get latest context
        context = self.get_context()

        # publish group information
        groups = context.interaction.get_groups()
        interaction_space_arr_msg = InteractionSpaceArray()
        for group in groups:
            msg = InteractionSpace()
            msg.center = Point(x=group['center'][0], y=group['center'][1], z=0)
            for a_p in group['agent_points']:
                msg.members.append(Point(x=a_p[0], y=a_p[1], z=a_p[2]))
            for h_p in group['human_points']:
                msg.members.append(Point(x=h_p[0], y=h_p[1], z=h_p[2]))
            msg.header = alloy.ros.create_ros_header(rospy, "env")
            interaction_space_arr_msg.spaces.append(msg)
        self._interaction_pub.publish(interaction_space_arr_msg)

    def get_context(self) -> Context:
        """Return the latest copy of the context

        Returns
        -------
        Context
            Describes the current context at the time point.
        """
        temp_copy = None
        with self._context_lock:
            temp_copy = copy.deepcopy(self._context)
        return self._context_processing(rospy.Time.now(), temp_copy)

    def _context_processing(self, _curr_time, copied_context):
        """Place holder for future operation that process the context before passing it out.
        """
        return copied_context
