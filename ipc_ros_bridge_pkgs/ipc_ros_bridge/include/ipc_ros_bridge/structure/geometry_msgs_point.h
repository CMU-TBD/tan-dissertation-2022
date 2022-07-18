#ifndef IPC_ROS_BRIDGE_STRUCTURE_GEOMETRY_MSGS_POINT_H
#define IPC_ROS_BRIDGE_STRUCTURE_GEOMETRY_MSGS_POINT_H

// Define the struct here
typedef struct 
{
    double x;
    double y;
    double z;
} geometry_msgs_point;

// Define the constants
#define GEOMETRY_MSGS_POINT_NAME "geometry_msgs_point"
#define GEOMETRY_MSGS_POINT_FORMAT "{double, double, double}"

#endif //IPC_ROS_BRIDGE_STRUCTURE_GEOMETRY_MSGS_POINT_H