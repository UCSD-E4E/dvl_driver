#include "ros/ros.h"
#include "std_msgs/String.h"
#include<auv_msgs/DvlOutput.h>
//#include<input.h>
#include<iostream>
#include <sstream>
using namespace std;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");            // "talker" is the name of the node
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<auv_msgs::DvlOutput>("chatter", 1000);
 	ros::Rate loop_rate(10);
// 	int count = 0;
  	while (ros::ok())
  	{
  		auv_msgs::DvlOutput msg;
//		msg.x = rand()%300; msg.y = rand()%300; msg.yaw = rand()%360;
		msg.vx = 1; msg.vy = 1; msg.vz = 1;
//		std::stringstream ss;
		ROS_INFO("Velocity is (%f,%f,%f)", msg.vx,msg.vy,msg.vz);
 		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
//	    ++count;
  	}
 	return 0;
}
