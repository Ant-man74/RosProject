# RosProject

Useful Command :

 - source devel/setup.bash
 - export ROS_MASTER_URI = http://192.168.1.34:11311
 - export ROS_IP = “your IP”;
 - catkin_make
 - roscore
 - rosrun package node

SUMMARY

follow me node:

- decision: decision_node
- perception: obstacle_detection_node
- perception: robot_moving_node
- perception: moving_person_detector_node
- action: rotation_action_node
- action: translation_action_node

Surveillance node:

- decision: decision_switch_behavior_node
- decision: exploration_node

STRUCTURE

Decision node:

- decision/decision_node
- decision/decision_switch_behavior_node
- decision/exploration_node

Perception node:

- perception/moving_person_detector_node
- perception/obstacle_detection_node

Action node:

- action/rotation_action_node
- action/translation_action_node
- action/robot_moving_node
