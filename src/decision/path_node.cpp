#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <string>

class path {

	private:
	    ros::NodeHandle n;

	     // communication with rotation_action
        ros::Publisher pub_rotation_to_do;
        ros::Subscriber sub_rotation_done;

        // communication with translation_action
	    ros::Publisher pub_translation_to_do;
	    ros::Subscriber sub_translation_done;

	    // communication with path
        ros::Publisher pub_switch_state_to_follow;
        ros::Subscriber sub_switched_state_to_path;
		
		//to retrieve current pose
        tf::TransformListener listener;
        tf::Transform transform;
        tf::StampedTransform stampedTransform;
        tf::TransformBroadcaster br;

		float allRotation;//keep track of all previous rotation to keep the orientation rigth	
		
	    float rotation_to_do;
	    float rotation_done;
	    float translation_to_do;
	    float translation_done;

	    geometry_msgs::Point currentPose;
	    geometry_msgs::Point point0;
	    geometry_msgs::Point point1;
	    geometry_msgs::Point point2;
	    geometry_msgs::Point point3;
	    geometry_msgs::Point point4;
	    geometry_msgs::Point point5;
	    geometry_msgs::Point point6;  
	    geometry_msgs::Point point7;

	    //path to do
	    geometry_msgs::Point pathList[PATHLENGTH]; // Array with the path	   
	    int currentPoint;//current point the robot is on	    
	    int orientation;//orientation of the robot

        geometry_msgs::Point goal_to_reach;
        geometry_msgs::Point goal_reached;
        
        bool new_rotation_done;//boolean to check if a rotation is done
        bool new_translation_done;//boolean to check if a translation is done

        bool firstTime;//boolean to check if it's the first turn
        bool adjusting;
        bool cond_rotation;//boolean to check if a rotation is going
        bool cond_translation;//boolean to check if a translation is going

        public:

            path() {		    

            	//robot need to start at point 0 and at an angle of around 150 degree (along y positive axis) because the graph is kind of s*** 
				point0.x = -11;
            	point0.y = 1;

				point1.x = -8.5;
            	point1.y = 7;

            	point2.x = -4;
            	point2.y = 4;

            	point3.x = -8.5;
            	point3.y = 7;

            	point4.x = -10;
            	point4.y = 4;

            	point5.x = -14;
            	point5.y = 5;

            	point6.x = -10;
            	point6.y = 4;

			    pathList[0] = point1;
			    pathList[1] = point2;
			    pathList[2] = point3;
			    pathList[3] = point4;
			    pathList[4] = point5;
			    pathList[5] = point6;
				currentPose = point0;

				//orientation :
            	/*
            	NORTH = 0
				EAST = 1
				SOUTh = 2				
				WEST = 3 
				I hate c++ enum would have been better but couldn't manage to do it
            	*/				
				allRotation = 0;
				orientation = 0;
                currentPoint = 0;

                //trying to set up to retreive the coordiante no idea if it worjk
                transform.setOrigin(tf::Vector3(5,0.0,0.0));
                transform.setRotation(tf::Quaternion(0,0,0,1));
				stampedTransform = tf::StampedTransform(transform,ros::Time::now(),"/odom","/dummyFrame");
                br.sendTransform(stampedTransform);

                // communication with rotation_action
                pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);
                sub_rotation_done = n.subscribe("rotation_done", 1, &path::rotation_doneCallback, this);

                // communication with translation_action
                pub_translation_to_do = n.advertise<std_msgs::Float32>("translation_to_do", 0);
                sub_translation_done = n.subscribe("translation_done", 1, &path::translation_doneCallback, this);
				
				// communication with decision node
        		pub_switch_state_to_follow = n.advertise<std_msgs::Float32>("switch_state_follow", 100);
				sub_switched_state_to_path = n.subscribe("switch_state_path", 1, &path::switch_stateCallback, this);

				new_rotation_done = false;
				new_translation_done = false;

				cond_translation = false;
				cond_rotation = false;

				firstTime = true;

                //loop to cycle through the path
                ros::Rate r(40);// this node will run at 10hz
                while (ros::ok()) {
                    ros::spinOnce();//each callback is called once to collect new data
                    update();//processing of data
                    r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
                }
            }

            //UPDATE: main processing
            /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
            void update() {
 				
 				ROS_INFO("(path_node update)");
               
                if( !cond_translation  && !cond_rotation && !adjusting){     	
	                   
                    goal_to_reach = pathList[currentPoint];                    

                    //because we don't start at 0,0 we have to calculate the relative distance between two point
	                float relativeX = currentPose.x - goal_to_reach.x; //-2.5
	                float relativeY = currentPose.y - goal_to_reach.y;//-6

	                //calculate translation
                	translation_to_do = sqrt((relativeX * relativeX) + (relativeY * relativeY));

                	ROS_INFO("(path_node (trans to do: %f)(currentRelative : %f, %f) )",translation_to_do, relativeX, relativeY);

                	//calculate rotation depending on orientation cf schema.png
                	switch(orientation){

                		case 0://orientation south
                			rotation_to_do = acos(relativeY/translation_to_do);ROS_INFO("a");
                			break;
                		case 1://orientation north
                			rotation_to_do = acos(relativeY/translation_to_do);	ROS_INFO("b");				
                			break;
						case 2://orientation EASR
							rotation_to_do = acos(relativeX/translation_to_do);ROS_INFO("c");
                			break;
						case 3://orientation WEST
							rotation_to_do = acos(relativeX/translation_to_do);	ROS_INFO("d");						
                			break;
                	}

					ROS_INFO("(path_node rotation to do) %f",rotation_to_do);
                	
					//avoid the first rotation because it's throwing things off I think
                	if(!firstTime){

                		//calculate new orientation based on all previous rotation we have done so far
                		allRotation = allRotation + rotation_to_do ;

                		float angleDegree = allRotation * (180.0/3.141592653589793238463);		

						//depending on all the previous rotation define the new orientation 
						if(angleDegree < 135 && angleDegree > 45){
							orientation = orientation-1;
						}else if(angleDegree > 135 && angleDegree < 225){
							orientation = orientation-2;
						}else if(angleDegree*-1 < 135 && angleDegree*-1 > 45){
							orientation = orientation+1;
						}else if(angleDegree*-1 > 135 && angleDegree*-1 < 225){
							orientation = orientation+2;
						}
						//else under 45 so don't change orientation
						
						//modulo 4 since we only have 2 orientation and abs cuz we don't want negative values
						orientation = abs(orientation % 4);
                	}

					firstTime = false;

                	//if the new rotation has not been done
                	if(!new_rotation_done){
                	
	                	ROS_INFO("(path_node) /rotation_to_do %f ",rotation_to_do);

                    	ROS_INFO("(path_node) %f %f",goal_to_reach.x, goal_to_reach.y);

	                	rotation_to_do = atan(goal_to_reach.x / goal_to_reach.y);

	                    std_msgs::Float32 msg_rotation_to_do;
	                    
	                    // publish rotate
	                    msg_rotation_to_do.data = rotation_to_do;
	                    pub_rotation_to_do.publish(msg_rotation_to_do);
	                   
	                    //declare that robot is turning
	                    cond_rotation = true;
					}

					//if the rotation is done and the translation has not been done
					if(new_rotation_done && !new_translation_done){
                		
                		ROS_INFO("(path_node) /translation_to_do %f ",translation_to_do);

						ROS_INFO("(path_node) %f %f",goal_to_reach.x, goal_to_reach.y);

	                    std_msgs::Float32 msg_translation_to_do;

	                    //publish translate
	                    msg_translation_to_do.data = translation_to_do;
	                    pub_translation_to_do.publish(msg_translation_to_do);

	                    //declare robot as moving
	                    cond_translation = true;

					} 
                    

					//if both the translation and rotation have been done
                    if(new_translation_done && new_rotation_done){

                    	ROS_INFO("(path_node) at aimed point %d ",currentPoint);                    	
                    	//both translation done
                    	new_rotation_done = false;
                    	new_translation_done = false;
                    	
                    	adjusting = true;

                    }

                    if(adjusting){

                    	adjusting = false;

                    	try{
			              	listener.waitForTransform("/base_link", "/dummyFrame", ros::Time(0), ros::Duration(3.0));
			              	listener.lookupTransform("/base_link", "/dummyFrame", ros::Time(0), stampedTransform);
			            }catch (tf::TransformException &ex) {
			              ROS_ERROR("%s",ex.what());
			              ros::Duration(1.0).sleep();
			            }

			           	ROS_INFO("current pose: %f", stampedTransform.getOrigin().y());
			           	
			           	currentPose.x = stampedTransform.getOrigin().x();
			           	currentPose.y = stampedTransform.getOrigin().y();
			           	
			           	//si le robot est a + ou moins 0.5 du goal on dit ok sinon pas bon
			           	if(!(currentPose.x-goal_to_reach.x < -0.5 || currentPose.x-goal_to_reach.x > 0.5 &&
			           		currentPose.y-goal_to_reach.y < -0.5 || currentPose.y-goal_to_reach.y > 0.5 )){
			           		//switch pointer to next point in patrol
                    		currentPoint++;		

                    		//If we are done with the last point put the counter back to 0 because we should be at origin
		                    if(currentPoint == 6){
		                    	currentPoint = 0;
		                    }
			           	}                   	
                    }
                }
            }

            //CALLBACKS
            /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

            void rotation_doneCallback(const std_msgs::Float32::ConstPtr& a) {
                // process the angle received from the rotation node
				ROS_INFO("(path_node) rotation callback");
                rotation_done = a->data;
				cond_rotation = false;
				new_rotation_done = true;
            }

            void translation_doneCallback(const std_msgs::Float32::ConstPtr& r) {
                // process the range received from the translation node
   				ROS_INFO("(path_node) translation callback");
                translation_done = r->data;
	            cond_translation = false;
				new_translation_done = true;

            }

            // Distance between two points
            float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
                return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
            }

};


int main(int argc, char **argv){

    ROS_INFO("(path_node) waiting /map_server");
    ros::init(argc, argv, "path");
    path bsObject;
    ros::spin();
    return 0;

}