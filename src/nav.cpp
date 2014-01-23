#include"ros/ros.h"
#include<iostream>
#include<auv_msgs/DvlOutput.h>
using namespace std;

float deltaT=0.1;		// give value here

struct V3
{
	float x,y,z;
	V3 modify(float vx,float vy,float vz)
	{
		V3 newV;
		newV.x = x + (vx * deltaT);
		newV.y = x + (vy * deltaT);
		newV.z = x + (vz * deltaT);
		return newV;
	}
};

V3 position={0,0,0};

void chatterCallback(const auv_msgs::DvlOutput& msg)
{
	position = position.modify(msg.vx,msg.vy,msg.vz);
	ROS_INFO("Velocity is (%f,%f,%f), and new position is (%f,%f,%f)",msg.vx,msg.vy,msg.vz,position.x,position.y,position.z);
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("chatter",1000,chatterCallback);
	ros::spin();
	return 0;
}
