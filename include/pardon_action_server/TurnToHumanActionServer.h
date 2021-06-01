#ifndef __TURN_TO_HUMAN_ACTION_SERVER_H__
#define __TURN_TO_HUMAN_ACTION_SERVER_H__

#include<exception>
#include<actionlib/server/simple_action_server.h>
#include<actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include<pardon_action_server/TurnToHumanAction.h>
#include<tf/transform_listener.h>
#include<twist_mux_msgs/JoyPriorityAction.h>
#include<std_msgs/String.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/JointState.h>
#include<geometry_msgs/Twist.h>

class InvalidParamException : public std::exception {
    const std::string paramName_;
  public:
    InvalidParamException(const std::string paramName);
    const char* what() const throw();
};

template<typename T>
T getParamValue(const std::string name){
    T value;
    if(!ros::param::get(name, value))
        throw InvalidParamException(name);
    return value;
}

class TurnToHumanActionServer {
  protected:
    ros::NodeHandle nh_;
    tf::TransformListener TFlistener;

    std::string actionName_;
    actionlib::SimpleActionServer<pardon_action_server::TurnToHumanAction> as_;
    pardon_action_server::TurnToHumanFeedback feedback_;
    pardon_action_server::TurnToHumanResult result_;

    actionlib::SimpleActionClient<twist_mux_msgs::JoyPriorityAction> ac_;

    ros::Subscriber odometrySub_;
    nav_msgs::Odometry currentOdom_;

    ros::Subscriber jointStateSub_;
    sensor_msgs::JointState currentJointState_;
    
    ros::Subscriber joyPrioritySub_;
    std_msgs::Bool currentJoyPriority_;

    ros::Publisher velocityPublisher_;
    
    void robotOdometryCallback(const nav_msgs::Odometry message);
    void robotJointStateCallback(const sensor_msgs::JointState message);
    void joyPriorityCallback(const std_msgs::Bool message);

    void publishFeedback(const std::string state);
    void publishStatus(const std::string state);
    void publishTorsoVelocityCommand(const double angularVelocity);
    void callJoyPriorityAction();

    const double findRequiredAngle() const;

  public:
    TurnToHumanActionServer();
    ~TurnToHumanActionServer();
    void executeCallback(const pardon_action_server::TurnToHumanGoalConstPtr &goal);                

    const std::string odometryTopic = "mobile_base_controller/odom";
    const std::string jointStateTopic = "joint_states";
    const std::string joyPriorityTopic = "joy_priority";
    const std::string joyPriorityAction = "joy_priority_action";
    const std::string velocityTopic = "joy_vel";
};

#endif