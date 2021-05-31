## TurnToHuman action definition

Goal definition: 
- geometry_msgs/PoseWithCovariance
  - position of a human that robot should turn to

Feedback definition:
- geometry_msgs/Quaternion
  - current orientation of microphone
- std_msgs/String
  - information about link that is rotating (head/torso)

Result definition:
- std_msgs/String
  - status of an finished action, information about which link has turned (head/torso)

## Topic subscriptions

Node currently listens to two topics:
- odometry (`nav_msgs/Odometry`, default: `/mobile_base_controller/odom`) - current position of the robot,
- jointStates (`sensor_msgs/JointState`, default: `/joint_states`) - current state of joints (we are interested in head state).

## Topic publications

Node publishes on one topic:
- velocity (`geometry_msgs/Twist`, default: `/key_vel`) - velocity commands to mobile base controller.  