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

## Called actions

Node currently calls two actions:
- joystickPriority (`twist_mux_msgs::JoyPriorityAction`, default: `/joy_priority_action`) - changes priority of joystick controller,
- pointHead (`control_msgs::PointHeadAction`, default: `/head_controller/point_head_action`) - move head to look at given point.

## Topic subscriptions

Node currently listens to two topics:
- odometry (`nav_msgs/Odometry`, default: `/mobile_base_controller/odom`) - current position of the robot,
- jointStates (`sensor_msgs/JointState`, default: `/joint_states`) - current state of joints (we are interested in head state),
- joyPriorityCallback(`std_msgs::Bool`, default: `/joy_priority`) - state of joystick priority.

## Topic publications

Node publishes on one topic:
- velocity (`geometry_msgs/Twist`, default: `/key_vel`) - velocity commands to mobile base controller.  

## Parameters
- `odometry_topic` - topic name to get odometry from robot,
- `joint_state_topic` - topic name to get robot's joint state,
- `joy_priority_topic` - topic name to get joystick priority,
- `command_velocity_topic` - topic name to publish velocities,
- `joy_priority_action` - action name to change joystick priority,
- `point_head_action` - action name to rotate head to point in given direction,
- `served_action_name` - name of action that this node serves,
- `use_joy_action` - flag whether node should handle joystick priority,
- `human_tf` - name of human frame,
- `torso_turning_velocity` - max velocity that should be used for turning the torso,
- `head_turning_velocity` - max velocity that should be used for turning the head,
- `base_link` - base link of the robot,
- `max_head_rotation` - maximum angle for head-only rotation.
