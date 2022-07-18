cmake_minimum_required(VERSION 3.0.2)
project(tbd_ros_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Agent.msg
  Actor.msg
  HumanJoint.msg
  HumanBody.msg
  HumanBodyArray.msg
  InteractionSpace.msg
  InteractionSpaceArray.msg
  FaceAnimationObject.msg
)


## Generate actions in the 'action' folder
 add_action_files(
   FILES
   faceEmotion.action
   faceAnimation.action
   playAudio.action
   pollySpeech.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   actionlib_msgs
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tbd_ros_msgs
  CATKIN_DEPENDS actionlib_msgs geometry_msgs sensor_msgs
#  DEPENDS system_lib 
)