#include<ros/ros.h>
#include<pardon_action_server/TurnToHumanActionServer.h>
#include<math.h>

InvalidParamException::InvalidParamException(const std::string paramName) : paramName_(paramName) {}

const char* InvalidParamException::what() const throw() {
    return paramName_.c_str();
}

void TurnToHumanActionServer::robotOdometryCallback(const nav_msgs::Odometry message){
    currentOdom_ = message;
}
    
void TurnToHumanActionServer::robotJointStateCallback(const sensor_msgs::JointState message){
    currentJointState_ = message;
}

void TurnToHumanActionServer::joyPriorityCallback(const std_msgs::Bool message){
    if(getParamValue<bool>("use_joy_action")){
        currentJoyPriority_ = message;
        ROS_INFO("current joy priority equals %s", currentJoyPriority_.data ? "true" : "false");
    }
}

void TurnToHumanActionServer::publishFeedback(const std::string state, const geometry_msgs::Quaternion orientation){
    std_msgs::String feedbackStatus;
    feedbackStatus.data = state;
    feedback_.status = feedbackStatus;
    feedback_.orientation = orientation;
    as_.publishFeedback(feedback_);
}

void TurnToHumanActionServer::publishStatus(const std::string state){
    std_msgs::String resultStatus;
    resultStatus.data = state;
    result_.status = resultStatus;
    as_.setSucceeded(result_);
    ROS_INFO("%s: Succeeded", actionName_.c_str());
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

TurnToHumanActionServer::TurnToHumanActionServer() : 
                      as_(nh_, getParamValue<std::string>("served_action_name"), boost::bind(&TurnToHumanActionServer::executeCallback, this, _1), false), 
                      ac_(getParamValue<std::string>("joy_priority_action"), true),
                      actionName_(getParamValue<std::string>("served_action_name")){
    odometrySub_ = nh_.subscribe(getParamValue<std::string>("odometry_topic"), 1000, &TurnToHumanActionServer::robotOdometryCallback, this);
    jointStateSub_ = nh_.subscribe(getParamValue<std::string>("joint_state_topic"), 1000, &TurnToHumanActionServer::robotJointStateCallback, this);
    joyPrioritySub_ = nh_.subscribe(getParamValue<std::string>("joy_priority_topic"), 1000, &TurnToHumanActionServer::joyPriorityCallback, this);

    velocityPublisher_ = nh_.advertise<geometry_msgs::Twist>(getParamValue<std::string>("command_velocity_topic"), 1000);
    
    as_.start();
    ROS_INFO("%s server ready", getParamValue<std::string>("served_action_name").c_str());

    if(getParamValue<bool>("use_joy_action")){
        ROS_INFO("joy priority action is being used");
        ac_.waitForServer();
        ROS_INFO("%s client ready", getParamValue<std::string>("joy_priority_action").c_str());
    }
}

TurnToHumanActionServer::~TurnToHumanActionServer(){}

void TurnToHumanActionServer::executeCallback(const pardon_action_server::TurnToHumanGoalConstPtr &goal){
    ROS_INFO("new goal requested");
    ros::Rate r(30);
    bool success = true;

    bool lockedState = currentJoyPriority_.data;
    if(!lockedState)
        callJoyPriorityAction();

    const double angle = findRequiredAngle();
    ROS_INFO("Desired change in yaw: %f degrees", (180*angle)/3.1415);

    for(auto i = 0; i <= 50; i++){
        if(as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", actionName_.c_str());
            as_.setPreempted();
            success = false;
            break;
        }

        geometry_msgs::Quaternion feedbackOrientation;
        feedbackOrientation = currentOdom_.pose.pose.orientation;

        publishTorsoVelocityCommand(0.0);
        publishFeedback("moving", feedbackOrientation);
        r.sleep();
    }
    publishTorsoVelocityCommand(0.0);

    if(!lockedState)
        callJoyPriorityAction();

    if(success)
        publishStatus("finished");
}

const double TurnToHumanActionServer::findRequiredAngle() const {
    tf::StampedTransform tf;
    this->TFlistener.lookupTransform(getParamValue<std::string>("human_tf"), "base_link", ros::Time(0), tf);
    tf::Transform tfi = tf.inverse();
    return atan2(tfi.getOrigin().y(), tfi.getOrigin().x());
}