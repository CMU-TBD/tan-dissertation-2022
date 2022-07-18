#pragma once
#include "../intermediate_type.h"
#include "geometry_msgs/Quaternion.h"
#include "../structure/geometry_msgs_quaternion.h"


class GoemetryMsgsQuaternion: public IntermediateType<geometry_msgs::Quaternion>
{
public:
    
    GoemetryMsgsQuaternion(const std::string name = GEOMETRY_MSGS_QUATERNION_NAME)
        :IntermediateType(name, GEOMETRY_MSGS_QUATERNION_FORMAT){
    }

    virtual void publishData(void* _msg){

        geometry_msgs::Quaternion *msg = (geometry_msgs::Quaternion *) _msg;
        geometry_msgs_quaternion quaternionStruct;
        quaternionStruct.x = msg->x;
        quaternionStruct.y = msg->y;
        quaternionStruct.z = msg->z;
        quaternionStruct.w = msg->w;
        sendToIPC(&quaternionStruct);
    }

    virtual geometry_msgs::Quaternion ContainerToMessage(void* _container)
    {
        geometry_msgs::Quaternion msg;
        // create new message
        geometry_msgs_quaternion *container = (geometry_msgs_quaternion *) _container;
        msg.x = container->x;
        msg.y = container->y;
        msg.z = container->z;
        msg.w = container->w;
        return msg;
    }
};
