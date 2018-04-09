#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

#define PATHLENGTH 7

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

	    float rotation_to_do;
	    float rotation_done;
	    float translation_to_do;
	    float translation_done;

	    geometry_msgs::Point point7;
	    geometry_msgs::Point point1;
	    geometry_msgs::Point point2;
	    geometry_msgs::Point point3;
	    geometry_msgs::Point point4;
	    geometry_msgs::Point point5;
	    geometry_msgs::Point point6;  

	    //path to do
	    geometry_msgs::Point pathList[PATHLENGTH]; // Array with the path
	   
	    int currentPoint;

        geometry_msgs::Point goal_to_reach;
        geometry_msgs::Point goal_reached;
        
        bool new_rotation_done;
        bool new_translation_done;
        bool atObjective;

        public:

            path() {
            	//origin x=-12, y = 0
            	//point 1 at x=-8.5		y=8
            	//point 2 at x=-3		y=5
            	//point 3 at x=-8.5 	y=8
            	//point 4 at x=-10	 	y=8
            	//point 5 at x=-14		y=8
            	//point 5 at x=-10		y=0
			    
			    point1.x = 3.5;
			    point1.y = 8;
			    
			    point2.x = 5.5;
			    point2.y = -3;
			    
			    point3.x = -5.5;
			    point3.y = 3;
			    
			    point4.x = -1.5;
			    point4.y = 0;
			    
			    point5.x = -4;
			    point5.y = 0;
			    
			    point6.x = 4;
			    point6.y = -8;

			    point7.x = -2;
			    point7.y = 0;

			    pathList[0] = point1;
			    pathList[1] = point2;
			    pathList[2] = point3;
			    pathList[3] = point4;
			    pathList[4] = point5;
			    pathList[5] = point6;
			    pathList[6] = point7;

                currentPoint = 0;
                atObjective = true;

                // communication with rotation_action
                pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);
                sub_rotation_done = n.subscribe("rotation_done", 1, &path::rotation_doneCallback, this);

                // communication with translation_action
                pub_translation_to_do = n.advertise<std_msgs::Float32>("translation_to_do", 0);
                sub_translation_done = n.subscribe("translation_done", 1, &path::translation_doneCallback, this);
						
        		//pub_switch_state_to_follow = n.advertise<std_msgs::Float32>("switch_state_follow", 100);
				//sub_switched_state_to_path = n.subscribe("switch_state_path", 1, &path::switch_stateCallback, this);

                //loop to cycle through the path
                ros::Rate r(10);// this node will run at 10hz
                while (currentPoint < PATHLENGTH-1) {
                    ros::spinOnce();//each callback is called once to collect new data
                    update();//processing of data
                    r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
                }
            }

            //UPDATE: main processing
            /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
            void update() {

                if( atObjective == true ){     	
	                   
	                    atObjective = false;
	                    currentPoint++;

   	                	ROS_INFO("(path_node update)");

                    	goal_to_reach = pathList[currentPoint+1];


                    	// Translation = sqrt(x^2 + y^2)
                   		translation_to_do = sqrt((goal_to_reach.x * goal_to_reach.x) + (goal_to_reach.y * goal_to_reach.y));

                    	// Rotation = arccos(x/abcysse_length)
                    	rotation_to_do = acos(goal_to_reach.x / translation_to_do);


	                    // Envoie l'info aux noeuds de rotation et de translation

	                    ROS_INFO("(path_node) /rotation_to_do %f ",rotation_to_do);
	                    std_msgs::Float32 msg_rotation_to_do;
	                    
	                    // publish rotate
	                    msg_rotation_to_do.data = rotation_to_do;
	                    pub_rotation_to_do.publish(msg_rotation_to_do);

	                    ROS_INFO("(path_node) /translation_to_do %f %f",goal_to_reach.x,goal_to_reach.y);
	                    std_msgs::Float32 msg_translation_to_do;
	                    
	                    //publish translate
	                    msg_translation_to_do.data = 1;
	                    pub_translation_to_do.publish(msg_translation_to_do);
	                     ROS_INFO("ok");

	                
                }
            }

            //CALLBACKS
            /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

            void rotation_doneCallback(const std_msgs::Float32::ConstPtr& a) {
                // process the angle received from the rotation node

                new_rotation_done = true;
                atObjective = true;
                rotation_done = a->data;

            }

            void translation_doneCallback(const std_msgs::Float32::ConstPtr& r) {
                // process the range received from the translation node
	                    ROS_INFO("(AAAAAAAAAAAAAAAAAAAAAAAAAAA");

                new_translation_done= true;
                atObjective = true;
                translation_done = r->data;

            }

            // Distance between two points
            float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
                return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
            }

            void switch_stateCallback(const std_msgs::Float32::ConstPtr& r) {
    		// process the change of state

    		}

};


int main(int argc, char **argv){

    ROS_INFO("(path_node) waiting /map_server");
    ros::init(argc, argv, "path");
    path bsObject;
    ros::spin();
    return 0;

}