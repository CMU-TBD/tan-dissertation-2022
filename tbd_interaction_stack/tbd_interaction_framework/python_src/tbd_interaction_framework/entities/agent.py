"""
Copyright - Transporation, Bots, and Disability Lab - Carnegie Mellon University
Released under MIT License

agent.py - Encodes an Agent.
"""

import typing

import numpy as np

import alloy.spatial.primitives


class Agent():
    """Encodes an Agent
    """

    id: str

    base: np.array  # (4,4) A representative position of the agent in the world
    attention_ray: np.array  # (3, ) ray that describe where the agent is looking at
    attention_origin: np.array  # (3,) that describe the origin of the attention
    bounding_box: alloy.spatial.primitives.Box  # Box

    # string-based dictionary for any additional use case specific information
    logbook: typing.Mapping[str, typing.Any]

    def __init__(self, id_, min_point=None, max_point=None):
        self.id = id_
        self.logbook = dict()
        self.attention_ray = np.zeros(3,)
        self.attention_origin = np.zeros(3,)
        self.base = np.eye(4)

        if min_point is not None:
            self.bounding_box = alloy.spatial.primitives.Box(min_point, max_point)
