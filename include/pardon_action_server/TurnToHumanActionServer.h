#ifndef __PARDON_ACTION_SERVER__TURN_TO_HUMAN_ACTION_SERVER_H__
#define __PARDON_ACTION_SERVER__TURN_TO_HUMAN_ACTION_SERVER_H__

#include<actionlib/server/simple_action_server.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/client/terminal_state.h>
#include<pardon_action_server/TurnToHumanAction.h>
#include<twist_mux_msgs/JoyPriorityAction.h>
#include<control_msgs/PointHeadAction.h>

#include<std_msgs/String.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/JointState.h>

#include<tf/transform_listener.h>

class TurnToHumanActionServer {
  protected:
    ros::NodeHandle nh_;
    tf::TransformListener TFlistener;

    std::string actionName_;
    actionlib::SimpleActionServer<pardon_action_server::TurnToHumanAction> as_;
    pardon_action_server::TurnToHumanFeedback feedback_;
    pardon_action_server::TurnToHumanResult result_;

    actionlib::SimpleActionClient<twist_mux_msgs::JoyPriorityAction> acJoy_;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> acHead_;

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
    bool moveTorso();
    bool moveHead();
    void resetHead();

  public:
    TurnToHumanActionServer();
    ~TurnToHumanActionServer();
    void executeCallback(const pardon_action_server::TurnToHumanGoalConstPtr &goal);
};

#endif