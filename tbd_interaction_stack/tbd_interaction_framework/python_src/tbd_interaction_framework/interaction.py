"""
Copyright - Transporation, Bots, and Disability Lab - Carnegie Mellon University
Released under MIT License

interaction.py - Encodes interaction properties of active entities.
"""

import typing

import numpy as np
import alloy.math

from .entities.human import Human
from .entities.agent import Agent


INTERACTION_START_BOUNDARY = 2.5
INTERACTION_LEAVE_BOUNDARY = 3


class Interaction():
    """ Object that tracks and keep track of interaction properties in the context.
    """

    _human_state: typing.Dict
    _space_state: typing.Dict

    def __init__(self):
        self._human_state = {}
        self._space_state = {}

    def get_groups(self) -> typing.List[typing.Dict]:
        """ Gets the groups that the Interacting is tracking.

        Returns:
            typing.List[typing.Dict]: list of groups. Each group is a string dictionary.
        """
        groups = []
        for group in self._space_state.values():
            groups.append(group)
        return groups

    def get_human_state(self, hid: str) -> str:
        """Get the state of the human with the associated ID.

        Args:
            hid (str): human id.

        Returns:
            str: state of the human in the interaction (Interactor, Observer, or Bystander)
        """
        return self._human_state.get(hid, None)

    def update(self, main_agent: Agent,
               active_agent_dict: typing.Mapping[str, Agent],
               active_human_list: typing.List[Human]) -> None:
        """ Update the interaction to detect and track groups.

        Args:
            main_agent (Agent): the main agent.
            active_agent_dict (typing.Mapping[str, Agent]): dictionary of agents in the context.
            active_human_list (typing.List[Human]): list of active humans.
        """

        active_human_dict = dict(zip([h.id for h in active_human_list], active_human_list))
        seen_human = []
        if main_agent is not None:
            # given proximity change the state of each tracked person
            for human in active_human_dict.values():
                seen_human.append(human.id)
                # calculate humans property relative to the main agent
                angle_to_main_agent = main_agent.base[:2, 3] - human.base[:2, 3]
                human_direction = alloy.math.normalize(human.base[:2, 0])
                facing_agent_thea = alloy.math.find_rotation(angle_to_main_agent, human_direction)
                dist_to_main_agent = np.sqrt(np.sum((main_agent.base[:2, 3] - human.base[:2, 3])**2))
                # if we never seen them before:
                if human.id not in self._human_state:
                    # if the person is sort of looking at the robot:
                    if facing_agent_thea > np.pi/3 or facing_agent_thea < -np.pi/3:
                        # if they are in the social interaction space boundaries
                        if dist_to_main_agent < INTERACTION_START_BOUNDARY:
                            self._human_state[human.id] = "Interactor"
                        else:
                            self._human_state[human.id] = "Observer"
                    else:
                        self._human_state[human.id] = "Bystandar"
                else:
                    if self._human_state[human.id] == "Interactor":
                        if dist_to_main_agent > INTERACTION_LEAVE_BOUNDARY:
                            if -np.pi/3 < facing_agent_thea < np.pi/3:
                                self._human_state[human.id] = "Observer"
                            else:
                                self._human_state[human.id] = "Bystander"
                    else:
                        # if the person is stu sort of looking at the robot:
                        if -np.pi/3 < facing_agent_thea < np.pi/3:
                            # if they are in the social interaction space boundaries
                            if dist_to_main_agent < INTERACTION_START_BOUNDARY:
                                self._human_state[human.id] = "Interactor"
                            else:
                                self._human_state[human.id] = "Observer"
                        else:
                            self._human_state[human.id] = "Bystandar"

                # if the user is an Interactor, we also calculate the O-Space
                if self._human_state[human.id] == "Interactor":

                    # we first see if any other agents can potentially be in the same O-space
                    # this is a crude number hack with heuristics
                    potential_agents = [main_agent.id]
                    for agent in active_agent_dict.values():
                        if agent.id == main_agent.id:
                            continue
                        dist_to_agent = np.sqrt(
                            np.sum((agent.base[:2, 3] - active_human_dict[human.id].base[:2, 3])**2))
                        if dist_to_agent < dist_to_main_agent:
                            # it also shouldn't be further from the main agent
                            dist_agent_to_main_agent = np.sqrt(np.sum((agent.base[:2, 3] - main_agent.base[:2, 3])**2))
                            if dist_agent_to_main_agent < dist_to_main_agent:
                                potential_agents.append(agent.id)

                    # now we try to generate O-space with all agents
                    # first calculate the centroid
                    center_point = active_human_dict[human.id].base[:2, 3].copy()
                    for agent_key in potential_agents:
                        agent = active_agent_dict[agent_key]
                        center_point[0] += agent.base[0, 3]
                        center_point[1] += agent.base[1, 3]
                    center_point = center_point/(len(potential_agents) + 1)

                    # add to the space information
                    self._space_state[human.id] = {
                        'center': center_point,
                        'humans': [human.id],
                        'human_points': [active_human_dict[human.id].base[:3, 3].copy()],
                        'agents': potential_agents,
                        'agent_points': [active_agent_dict[a].base[:3, 3].copy() for a in potential_agents]
                    }
                else:
                    # remocing human
                    if human.id in self._space_state:
                        del self._space_state[human.id]
            # remove if human doesn't exist
            for key in list(self._space_state.keys()):
                if key not in seen_human:
                    del self._space_state[key]
