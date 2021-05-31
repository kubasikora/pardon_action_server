#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<pardon/TurnToHumanAction.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Quaternion.h>

class TurnToHumanAction {
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<pardon::TurnToHumanAction> as_;
    std::string actionName_;
    pardon::TurnToHumanFeedback feedback_;
    pardon::TurnToHumanResult result_;

  public:
    TurnToHumanAction(std::string name) : as_(nh_, name, boost::bind(&TurnToHumanAction::executeCallback, this, _1), false), actionName_(name){
        as_.start();
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
            feedbackOrientation.x = 1.0 * i;
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