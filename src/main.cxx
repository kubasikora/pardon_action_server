#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "pardon");
    std::cout << "Hello world!" << std::endl;
    return 0;
}