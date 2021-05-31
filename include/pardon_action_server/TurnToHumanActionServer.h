#ifndef __TURN_TO_HUMAN_ACTION_SERVER_H__
#define __TURN_TO_HUMAN_ACTION_SERVER_H__

#include<actionlib/server/simple_action_server.h>
#include<pardon_action_server/TurnToHumanAction.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/JointState.h>
#include<geometry_msgs/Twist.h>

class TurnToHumanActionServer {
  protected:
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<pardon_action_server::TurnToHumanAction> as_;
    std::string actionName_;
    pardon_action_server::TurnToHumanFeedback feedback_;
    pardon_action_server::TurnToHumanResult result_;

    ros::Subscriber odometrySub_;
    nav_msgs::Odometry currentOdom_;

    ros::Subscriber jointStateSub_;
    sensor_msgs::JointState currentJointState_;
    
    ros::Publisher velocityPublisher_;

    void robotOdometryCallback(const nav_msgs::Odometry message);
    void robotJointStateCallback(const sensor_msgs::JointState message);

  public:
    TurnToHumanActionServer(std::string name, std::string odometryTopic = "mobile_base_controller/odom", std::string jointStateTopic = "joint_states", std::string velocityTopic = "key_vel");
    ~TurnToHumanActionServer();
    void executeCallback(const pardon_action_server::TurnToHumanGoalConstPtr &goal);                
};

#endif