#include<ros/ros.h>
#include<pardon_action_server/TurnToHumanActionServer.h>

void TurnToHumanActionServer::robotOdometryCallback(const nav_msgs::Odometry message){
    currentOdom_ = message;
}
    
void TurnToHumanActionServer::robotJointStateCallback(const sensor_msgs::JointState message){
    currentJointState_ = message;
}

TurnToHumanActionServer::TurnToHumanActionServer(std::string name, std::string odometryTopic, std::string jointStateTopic, std::string velocityTopic) : 
                      as_(nh_, name, boost::bind(&TurnToHumanActionServer::executeCallback, this, _1), false), 
                      actionName_(name){
    as_.start();
    odometrySub_ = nh_.subscribe(odometryTopic, 1000, &TurnToHumanActionServer::robotOdometryCallback, this);
    jointStateSub_ = nh_.subscribe(jointStateTopic, 1000, &TurnToHumanActionServer::robotJointStateCallback, this);
    velocityPublisher_ = nh_.advertise<geometry_msgs::Twist>(velocityTopic, 1000);
}

TurnToHumanActionServer::~TurnToHumanActionServer(){}

void TurnToHumanActionServer::executeCallback(const pardon_action_server::TurnToHumanGoalConstPtr &goal){
    ROS_INFO("New goal requested");
    ros::Rate r(30);
    bool success = true;

    for(auto i = 0; i <= 50; i++){
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
        feedbackOrientation = currentOdom_.pose.pose.orientation;
        feedback_.orientation = feedbackOrientation;

        geometry_msgs::Twist newVelocity;
        newVelocity.angular.z = 0.5;
        velocityPublisher_.publish(newVelocity);

        as_.publishFeedback(feedback_);
        r.sleep();
    }

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