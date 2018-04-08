# RosProject

Useful Command :

	- source devel/setup.bash
	- export ROS_MASTER_URI = http://192.168.1.34:11311
	- export ROS_IP = 192.168.1.10;
	- catkin_make
	- roscore
	- rosrun package node
	- rosrun map_server map_server maps/fablab.yaml
	- rosrun amcl amcl
  
Dans rviz:
	
	- Ajouter LaserScan, topic /scan
	- Ajouter Map, topic /map
	- Ajouter PoseWithCovariance, topic /amcl_pose
	- Ajouter PoseArray, /particlecloud

Nodes:

	- decision: decision_node
	- decision: path_node
	- perception: obstacle_detection_node
	- perception: robot_moving_node
	- perception: moving_person_detector_node
	- action: rotation_action_node
	- action: translation_action_node
