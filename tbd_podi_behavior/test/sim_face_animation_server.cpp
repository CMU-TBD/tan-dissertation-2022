#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tbd_ros_msgs/faceAnimationAction.h>

class FakeAnimationServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<tbd_ros_msgs::faceAnimationAction> as_;
    // create messages that are used to published feedback/result
    tbd_ros_msgs::faceAnimationFeedback feedback_;
    tbd_ros_msgs::faceAnimationResult result_;

public:
    FakeAnimationServer() : as_(nh_, "animation", boost::bind(&FakeAnimationServer::executeCB, this, _1), false)
    {
        as_.start();
    }

    void executeCB(const tbd_ros_msgs::faceAnimationGoalConstPtr &goal)
    {
        as_.setSucceeded(result_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_fake_animation_server");
    FakeAnimationServer fakeServer;
    ros::spin();
}