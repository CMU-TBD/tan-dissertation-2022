#ifndef IPC_ROS_BRIDGE_STRUCTURE_GEOMETRY_MSGS_QUATERNION_H
#define IPC_ROS_BRIDGE_STRUCTURE_GEOMETRY_MSGS_QUATERNION_H

// Define the struct here
typedef struct 
{
    double x;
    double y;
    double z;
    double w;
} geometry_msgs_quaternion;

// Define the constants
#define GEOMETRY_MSGS_QUATERNION_NAME "geometry_msgs_quaternion"
#define GEOMETRY_MSGS_QUATERNION_FORMAT "{double, double, double, double}"

#endif //IPC_ROS_BRIDGE_STRUCTURE_GEOMETRY_MSGS_QUATERNION_H