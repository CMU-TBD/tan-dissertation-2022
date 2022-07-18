#pragma once
#include "../intermediate_type.h"
#include "geometry_msgs/Point.h"
#include "../structure/geometry_msgs_point.h"


class GoemetryMsgsPoint: public IntermediateType<geometry_msgs::Point>
{
public:
    
    GoemetryMsgsPoint(const std::string name = GEOMETRY_MSGS_POINT_NAME)
        :IntermediateType(name, GEOMETRY_MSGS_POINT_FORMAT){
    }

    virtual void publishData(void* _msg){

        geometry_msgs::Point *msg = (geometry_msgs::Point *) _msg;
        geometry_msgs_point pointStruct;
        pointStruct.x = msg->x;
        pointStruct.y = msg->y;
        pointStruct.z = msg->z;
        sendToIPC(&pointStruct);
    }

    virtual geometry_msgs::Point ContainerToMessage(void* _container)
    {
        geometry_msgs::Point msg;
        // create new message
        geometry_msgs_point *container = (geometry_msgs_point *) _container;
        msg.x = container->x;
        msg.y = container->y;
        msg.z = container->z;
        return msg;
    }
};
