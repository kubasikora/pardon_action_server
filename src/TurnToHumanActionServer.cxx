#include<ros/ros.h>
#include<pardon/TurnToHumanActionServer.h>

void TurnToHumanActionServer::robotOdometryCallback(const nav_msgs::Odometry message){
    currentOdom_ = message;
    ROS_INFO("Updated odometry");
}
    
void TurnToHumanActionServer::robotJointStateCallback(const sensor_msgs::JointState message){
    currentJointState_ = message;
    ROS_INFO("Updated joints state");
}

TurnToHumanActionServer::TurnToHumanActionServer(std::string name, std::string odometryTopic, std::string jointStateTopic) : 
                      as_(nh_, name, boost::bind(&TurnToHumanActionServer::executeCallback, this, _1), false), 
                      actionName_(name){
    as_.start();
    odometrySub_ = nh_.subscribe(odometryTopic, 1000, &TurnToHumanActionServer::robotOdometryCallback, this);
    jointStateSub_ = nh_.subscribe(jointStateTopic, 1000, &TurnToHumanActionServer::robotJointStateCallback, this);
}

TurnToHumanActionServer::~TurnToHumanActionServer(){}

void TurnToHumanActionServer::executeCallback(const pardon::TurnToHumanGoalConstPtr &goal){
    ros::Rate r(1);
    bool success = true;

    for(auto i = 0; i <= 5; i++){
        if(as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", actionName_.c_str());
            as_.setPreempted();
            success = false;
            break;
        }

        std_msgs::String feedbackStatus;
        feedbackStatus.data = "moving";
        feedback_.status = feedbackStatus;

        geometry_msgs::Quaternion feedbackOrientation;
        feedbackOrientation.x = currentOdom_.pose.pose.position.x;
        feedback_.orientation = feedbackOrientation;

        as_.publishFeedback(feedback_);
        r.sleep();
    }

    if(success){
        std_msgs::String resultStatus;
        resultStatus.data = "moved";
        result_.status = resultStatus;

        ROS_INFO("%s: Succeeded", actionName_.c_str());
        as_.setSucceeded(result_);
    }
}