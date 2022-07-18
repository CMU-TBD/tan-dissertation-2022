# tbd_interaction_stack
COPYRIGHT(C) 2020 - Transportation, Bots, and Disability Lab - Carnegie Mellon University    
Code released under MIT   
Contact - Zhi - xiangzht@alumni.cmu.edu   

## Overview 
A collection of ROS Packages & Python Modules to enable fluid interaction between robots and users. The central idea of the framework is to:
1. Combine and merge different sources of sensor and interaction inputs through the `Merger` component.
2. Tools for Interactions such as Dialog Picker, etc
3. Ability to command the agent using high level intents such as `gazeAt`, `SpeakTo`, etc. These intents are model as actionlib topics.
