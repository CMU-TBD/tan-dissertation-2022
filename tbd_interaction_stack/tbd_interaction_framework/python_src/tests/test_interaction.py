
from tbd_interaction_framework.context import Context
from tbd_interaction_framework.entities.agent import Agent
from tbd_interaction_framework.entities.human import Human

from alloy.spatial.primitives import Box
import numpy as np


def make_fake_dyad_context(h_point, a_point):
    context = Context('main')

    # add fake human
    h = Human()
    h.base = np.eye(4)
    h.base[:3, 3] = h_point
    h.id = "0"
    h.bounding_box = Box([-0.2, -0.2, 0], [0.2, 0.2, 1])
    h.bounding_box.center = h_point
    h.focus_point = np.array([h_point[0], h_point[1], + h_point[2]+1])
    context.active_humans = [h]
    context.humans[h.id] = h
    # add fake baxter
    a = Agent("main")
    a.base[:3, 3] = a_point
    a.base[:3, 0] = [-1, 0, 0]
    a.base[:3, 1] = [0, -1, 0]
    a.bounding_box = Box([-0.1, 1.9, 0], [0.1, 2.1, 0])
    a.bounding_box.center = a_point
    context.agents[a.id] = a
    context.main_agent = a
    # update interactions
    context.interaction.update(a, context.agents, context.active_humans)
    return context


def test_interaction_group_detection():
    context = make_fake_dyad_context([0, 0, 0], [0, 2, 0])
    groups = context.interaction.get_groups()
    assert len(groups) == 1
    np.testing.assert_almost_equal(groups[0]['center'], [0, 1])
    assert len(groups[0]['human_points']) == 1
    assert len(groups[0]['agent_points']) == 1
    np.testing.assert_almost_equal(groups[0]['human_points'][0], [0, 0, 0])
    np.testing.assert_almost_equal(groups[0]['agent_points'][0], [0, 2, 0])


def test_interaction_people_move():

    context = make_fake_dyad_context([0, 0, 0], [0, 2, 0])
    groups = context.interaction.get_groups()
    assert len(groups) == 1
    np.testing.assert_almost_equal(groups[0]['center'], [0, 1])

    # move people
    context.active_humans[0].base[:3, 3] = [10, 10, 0]
    context.active_humans[0].bounding_box.center = [10, 10, 0]
    context.interaction.update(context.main_agent, context.agents, context.active_humans)
    groups = context.interaction.get_groups()
    assert len(groups) == 0
