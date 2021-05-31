#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<pardon/TurnToHumanAction.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/JointState.h>

class TurnToHumanAction {
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

    void robotOdometryCallback(const nav_msgs::Odometry message){
        currentOdom_ = message;
        ROS_INFO("Updated odometry");
    }
    
    void robotJointStateCallback(const sensor_msgs::JointState message){
        currentJointState_ = message;
        ROS_INFO("Updated joints state");
    }

  public:
    TurnToHumanAction(std::string name, 
                      std::string odometryTopic = "mobile_base_controller/odom", 
                      std::string jointStateTopic = "joint_states") : 
                      as_(nh_, name, boost::bind(&TurnToHumanAction::executeCallback, this, _1), false), 
                      actionName_(name){
        as_.start();
        odometrySub_ = nh_.subscribe(odometryTopic, 1000, &TurnToHumanAction::robotOdometryCallback, this);
        jointStateSub_ = nh_.subscribe(jointStateTopic, 1000, &TurnToHumanAction::robotJointStateCallback, this);
    }

    ~TurnToHumanAction(){}

    void executeCallback(const pardon::TurnToHumanGoalConstPtr &goal){
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
};

int main(int argc, char** argv){
    ros::init(argc, argv, "pardon");
    
    TurnToHumanAction actionServer("pardon");
    ros::spin();

    return 0;
}