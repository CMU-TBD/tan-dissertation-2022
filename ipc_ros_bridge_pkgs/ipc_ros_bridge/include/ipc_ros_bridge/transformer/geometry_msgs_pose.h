#pragma once
#include "../intermediate_type.h"
#include "geometry_msgs/Pose.h"
#include "../structure/geometry_msgs_pose.h"


class GoemetryMsgsPose: public IntermediateType<geometry_msgs::Pose>
{
public:
    
    GoemetryMsgsPose(const std::string name = GEOMETRY_MSGS_POSE_NAME)
        :IntermediateType(name, GEOMETRY_MSGS_POSE_FORMAT){
    }

    virtual void publishData(void* _msg){

        geometry_msgs::Pose *msg = (geometry_msgs::Pose *) _msg;
        geometry_msgs_pose poseStruct;
        poseStruct.orientation.x = msg->orientation.x;
        poseStruct.orientation.y = msg->orientation.y;
        poseStruct.orientation.z = msg->orientation.z;
        poseStruct.orientation.w = msg->orientation.w;
        poseStruct.position.x = msg->position.x;
        poseStruct.position.y = msg->position.y;
        poseStruct.position.z = msg->position.z;
        sendToIPC(&poseStruct);
    }

    virtual geometry_msgs::Pose ContainerToMessage(void* _container)
    {
        geometry_msgs::Pose msg;
        // create new message
        geometry_msgs_pose *container = (geometry_msgs_pose *) _container;
        msg.orientation.x = container->orientation.x;
        msg.orientation.y = container->orientation.y;
        msg.orientation.z = container->orientation.z;
        msg.orientation.w = container->orientation.w;
        msg.position.x = container->position.x;
        msg.position.y = container->position.y;
        msg.position.z = container->position.z;
        return msg;
    }
};
