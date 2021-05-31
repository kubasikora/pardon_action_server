#ifndef __TURN_TO_HUMAN_ACTION_SERVER_H__
#define __TURN_TO_HUMAN_ACTION_SERVER_H__

#include<actionlib/server/simple_action_server.h>
#include<pardon/TurnToHumanAction.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/JointState.h>

class TurnToHumanActionServer {
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<pardon::TurnToHumanAction> as_;
    std::string actionName_;
    pardon::TurnToHumanFeedback feedback_;
    pardon::TurnToHumanResult result_;

    ros::Subscriber odometrySub_;
    nav_msgs::Odometry currentOdom_;

    ros::Subscriber jointStateSub_;
    sensor_msgs::JointState currentJointState_;
    
    void robotOdometryCallback(const nav_msgs::Odometry message);
    void robotJointStateCallback(const sensor_msgs::JointState message);

  public:
    TurnToHumanActionServer(std::string name, std::string odometryTopic = "mobile_base_controller/odom", std::string jointStateTopic = "joint_states");
    ~TurnToHumanActionServer();
    void executeCallback(const pardon::TurnToHumanGoalConstPtr &goal);                
};

#endif