#include<ros/ros.h>
#include<pardon_action_server/TurnToHumanActionServer.h>

void TurnToHumanActionServer::robotOdometryCallback(const nav_msgs::Odometry message){
    currentOdom_ = message;
}
    
void TurnToHumanActionServer::robotJointStateCallback(const sensor_msgs::JointState message){
    currentJointState_ = message;
}

void TurnToHumanActionServer::joyPriorityCallback(const std_msgs::Bool message){
    currentJoyPriority_ = message;
    ROS_INFO("Current joy priority equals %s", currentJoyPriority_.data ? "true" : "false");
}

void TurnToHumanActionServer::publishFeedback(const std::string state, const geometry_msgs::Quaternion orientation){
    std_msgs::String feedbackStatus;
    feedbackStatus.data = state;
    feedback_.status = feedbackStatus;
    feedback_.orientation = orientation;
    as_.publishFeedback(feedback_);
}

void TurnToHumanActionServer::publishTorsoVelocityCommand(const double angularVelocity){
    geometry_msgs::Twist newVelocity;
    newVelocity.angular.z = angularVelocity;
    velocityPublisher_.publish(newVelocity);
}

void TurnToHumanActionServer::callJoyPriorityAction(){
    twist_mux_msgs::JoyPriorityGoal goal;
    ac_.sendGoal(goal);
}

TurnToHumanActionServer::TurnToHumanActionServer(std::string name) : 
                      as_(nh_, name, boost::bind(&TurnToHumanActionServer::executeCallback, this, _1), false), 
                      ac_("joy_priority_action", true),
                      actionName_(name){
    odometrySub_ = nh_.subscribe(odometryTopic, 1000, &TurnToHumanActionServer::robotOdometryCallback, this);
    jointStateSub_ = nh_.subscribe(jointStateTopic, 1000, &TurnToHumanActionServer::robotJointStateCallback, this);
    joyPrioritySub_ = nh_.subscribe(joyPriorityTopic, 1000, &TurnToHumanActionServer::joyPriorityCallback, this);
    
    velocityPublisher_ = nh_.advertise<geometry_msgs::Twist>(velocityTopic, 1000);
    
    as_.start();
    ROS_INFO("%s server ready", name.c_str());

    ac_.waitForServer();
    ROS_INFO("%s client ready", joyPriorityAction.c_str());
}

TurnToHumanActionServer::~TurnToHumanActionServer(){}

void TurnToHumanActionServer::executeCallback(const pardon_action_server::TurnToHumanGoalConstPtr &goal){
    ROS_INFO("New goal requested");
    ros::Rate r(30);
    bool success = true;

    bool lockedState = currentJoyPriority_.data;
    if(!lockedState)
        callJoyPriorityAction();

    for(auto i = 0; i <= 50; i++){
        if(as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", actionName_.c_str());
            as_.setPreempted();
            success = false;
            break;
        }

        geometry_msgs::Quaternion feedbackOrientation;
        feedbackOrientation = currentOdom_.pose.pose.orientation;

        publishTorsoVelocityCommand(0.5);
        publishFeedback("moving", feedbackOrientation);
        r.sleep();
    }

    if(!lockedState)
        callJoyPriorityAction();

    if(success){
        std_msgs::String resultStatus;
        resultStatus.data = "moved";
        result_.status = resultStatus;

        geometry_msgs::Twist newVelocity;
        newVelocity.angular.z = 0.0;
        velocityPublisher_.publish(newVelocity);

        ROS_INFO("%s: Succeeded", actionName_.c_str());
        as_.setSucceeded(result_);
    }
}