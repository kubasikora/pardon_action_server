#include<ros/ros.h>
#include<pardon/TurnToHumanActionServer.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "pardon");
    std::string actionName = "pardon";
    
    TurnToHumanActionServer actionServer(actionName);
    ros::spin();

    return 0;
}