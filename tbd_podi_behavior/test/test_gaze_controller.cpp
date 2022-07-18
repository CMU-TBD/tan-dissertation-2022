#include <ros/ros.h>
#include <gtest/gtest.h>
#include <actionlib/client/simple_action_client.h>
#include <tbd_interaction_msgs/gazeAtAction.h>
#include <tbd_ros_msgs/faceAnimationAction.h>


TEST(PodiGazeActuator, ConnectionTest)
{

    actionlib::SimpleActionClient<tbd_interaction_msgs::gazeAtAction> client("gazeAtActuator", true);
    client.waitForServer();
    tbd_interaction_msgs::gazeAtGoal goal;
    
    client.sendGoal(goal);
    ASSERT_TRUE(client.waitForResult(ros::Duration(5.0)));
    auto state = client.getState();
    ASSERT_TRUE(state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED);
    ASSERT_TRUE(true);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "podi_gaze_unittest");
    return RUN_ALL_TESTS();
}
