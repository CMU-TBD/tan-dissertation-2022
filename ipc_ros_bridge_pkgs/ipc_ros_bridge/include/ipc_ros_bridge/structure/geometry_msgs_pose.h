#ifndef IPC_ROS_BRIDGE_STRUCTURE_GEOMETRY_MSGS_POSE_H
#define IPC_ROS_BRIDGE_STRUCTURE_GEOMETRY_MSGS_POSE_H

#include "geometry_msgs_point.h"
#include "geometry_msgs_quaternion.h"

// Define the struct here
typedef struct 
{
    geometry_msgs_point position;
    geometry_msgs_quaternion orientation;
} geometry_msgs_pose;

// Define the constants
#define GEOMETRY_MSGS_POSE_NAME "geometry_msgs_quaternion"
#define GEOMETRY_MSGS_POSE_FORMAT "{{double, double, double},{double, double, double, double}}"

#endif //IPC_ROS_BRIDGE_STRUCTURE_GEOMETRY_MSGS_POSE_H