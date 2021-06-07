#include<ros/ros.h>
#include<iostream>
#include<pardon_action_server/TurnToHumanActionServer.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "pardon");
    TurnToHumanActionServer actionServer;
    ROS_INFO("Action server ready to roll");
    ros::spin();
    return 0;
}