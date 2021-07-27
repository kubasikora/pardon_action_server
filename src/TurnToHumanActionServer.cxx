#include<ros/ros.h>
#include<pardon_action_server/TurnToHumanActionServer.h>
#include<pardon_action_server/util.h>
#include<math.h>

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

void TurnToHumanActionServer::publishFeedback(const std::string state){
    std_msgs::String feedbackStatus;
    feedbackStatus.data = state;
    feedback_.status = feedbackStatus;

    geometry_msgs::Quaternion feedbackOrientation;
    feedbackOrientation = currentOdom_.pose.pose.orientation;
    feedback_.orientation = feedbackOrientation;
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
    acJoy_.sendGoal(goal);
}

const double TurnToHumanActionServer::findRequiredAngle() const {
    tf::StampedTransform tf;
    this->TFlistener.lookupTransform(getParamValue<std::string>("human_tf"), getParamValue<std::string>("base_link"), ros::Time(0), tf);
    tf::Transform tfi = tf.inverse();
    return atan2(tfi.getOrigin().y(), tfi.getOrigin().x());
}

bool TurnToHumanActionServer::moveTorso(){
    bool success = true;
    bool lockedState = currentJoyPriority_.data;
    ros::Rate r(30);
    
    const double velocity = getParamValue<double>("torso_turning_velocity");
    const double initialAngle = findRequiredAngle();

    resetHead();

    if(!lockedState)
        callJoyPriorityAction();

    while(ros::ok()){
        if(as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", actionName_.c_str());
            as_.setPreempted();
            acHead_.stopTrackingGoal();
            success = false;
            break;
        }

        const double angleChange = findRequiredAngle();
        if(abs(angleChange) < 0.05 || angleChange*initialAngle < 0){
            publishTorsoVelocityCommand(0.0);
            break;
        }
  
        publishTorsoVelocityCommand(angleChange > 0.0 ? velocity : -velocity);
        publishFeedback("moving");
        r.sleep();
    }
    publishTorsoVelocityCommand(0.0);

    if(!lockedState)
        callJoyPriorityAction();

    return success;
}

geometry_msgs::Pose TurnToHumanActionServer::getPose(const std::string point, const std::string origin){
    tf::StampedTransform transformation;
    TFlistener.lookupTransform(point, origin, ros::Time(0), transformation);
    tf::Transform inversedTransformation = transformation.inverse();

    geometry_msgs::Pose pose;
    pose.position.x    = inversedTransformation.getOrigin().x();
    pose.position.y    = inversedTransformation.getOrigin().y();
    pose.position.z    = inversedTransformation.getOrigin().z();
    pose.orientation.x = inversedTransformation.getRotation().x();
    pose.orientation.y = inversedTransformation.getRotation().y();
    pose.orientation.z = inversedTransformation.getRotation().z();
    pose.orientation.w = inversedTransformation.getRotation().w();
    return pose;
}

bool TurnToHumanActionServer::moveHead(){
    control_msgs::PointHeadGoal goal;
    geometry_msgs::Pose pose = getPose(getParamValue<std::string>("human_tf"), "base_link");
    goal.target.header.frame_id = getParamValue<std::string>("base_link");
    goal.target.point.x         = pose.position.x; 
    goal.target.point.y         = pose.position.y; 
    goal.target.point.z         = pose.position.z;
    goal.pointing_axis.x        = 0.0; 
    goal.pointing_axis.y        = 0.0; 
    goal.pointing_axis.z        = 1.0;
    goal.pointing_frame         = getParamValue<std::string>("/head_controller/point_head_action/tilt_link");
    goal.max_velocity           = getParamValue<double>("head_turning_velocity");
    acHead_.sendGoal(goal);
    
    bool success = true;
    while(ros::ok()){
        if(as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", actionName_.c_str());
            as_.setPreempted();
            success = false;
            break;
        }
        publishFeedback("moving");
        actionlib::SimpleClientGoalState state = acHead_.getState();
        if(state.isDone()){
            if(state.toString() != "SUCCEEDED"){
                success = false;
            }
            break;
        }
    }

    return success;
}

void TurnToHumanActionServer::resetHead(){
    control_msgs::PointHeadGoal goal;
    goal.target.header.frame_id = "base_footprint";
    goal.target.point.x = 1.0; goal.target.point.y = 0.0; goal.target.point.z = 1.0;
    goal.pointing_axis.x = 1.0; goal.pointing_axis.y = 0.0; goal.pointing_axis.z = 0.0;
    goal.pointing_frame = getParamValue<std::string>("/head_controller/point_head_action/tilt_link");
    goal.max_velocity = getParamValue<double>("head_turning_velocity");
    acHead_.sendGoal(goal);
}

TurnToHumanActionServer::TurnToHumanActionServer() : 
                      as_(nh_, getParamValue<std::string>("served_action_name"), boost::bind(&TurnToHumanActionServer::executeCallback, this, _1), false), 
                      acJoy_(getParamValue<std::string>("joy_priority_action"), true),
                      acHead_(getParamValue<std::string>("point_head_action"), true),
                      actionName_(getParamValue<std::string>("served_action_name")){
    odometrySub_ = nh_.subscribe(getParamValue<std::string>("odometry_topic"), 1000, &TurnToHumanActionServer::robotOdometryCallback, this);
    jointStateSub_ = nh_.subscribe(getParamValue<std::string>("joint_state_topic"), 1000, &TurnToHumanActionServer::robotJointStateCallback, this);
    joyPrioritySub_ = nh_.subscribe(getParamValue<std::string>("joy_priority_topic"), 1000, &TurnToHumanActionServer::joyPriorityCallback, this);
    velocityPublisher_ = nh_.advertise<geometry_msgs::Twist>(getParamValue<std::string>("command_velocity_topic"), 1000);
    
    as_.start();
    ROS_INFO("%s server ready", getParamValue<std::string>("served_action_name").c_str());

    acHead_.waitForServer();
    ROS_INFO("%s client ready", getParamValue<std::string>("point_head_action").c_str());

    if(getParamValue<bool>("use_joy_action")){
        ROS_INFO("joy priority action is being used");
        acJoy_.waitForServer();
        ROS_INFO("%s client ready", getParamValue<std::string>("joy_priority_action").c_str());
    }
}

TurnToHumanActionServer::~TurnToHumanActionServer(){}

void TurnToHumanActionServer::executeCallback(const pardon_action_server::TurnToHumanGoalConstPtr &goal){
    ROS_INFO("new goal requested");
    bool success = true;

    const double angle = findRequiredAngle();
    ROS_INFO("Desired change in yaw: %f degrees", (180*angle)/3.1415);
    const double maxHeadRotationRadian = getParamValue<double>("max_head_rotation");

    if(std::abs(angle) > maxHeadRotationRadian){
        ROS_INFO("Moving torso");
        success = moveTorso();
    }
    else {
        ROS_INFO("Moving head");
        success = moveHead();
    }
    
    if(success)
        publishStatus("finished");
    else
        as_.setAborted();
}
