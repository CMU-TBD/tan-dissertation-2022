#!/usr/bin/env python3

import rospy
from tbd_interaction_framework.merger import ContextMerger

def main():
    rospy.init_node("test_framework", log_level=rospy.DEBUG)

    main_agent = "baxter"

    merger = ContextMerger(
        {
        'agent_types':{
            'podi':'podi.PodiAgentState',
            'baxter':'baxter.BaxterAgentState',
            'robocept':'robocept.RoboceptAgentState'
        }},
        main_agent
    )
    merger.start_listening()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        context = merger.get_context()
        groups = context.interaction.get_groups()
        for g in groups:
            print(g['center'], g['humans'], g['agents'])
        r.sleep()

if __name__ == "__main__":
    main()