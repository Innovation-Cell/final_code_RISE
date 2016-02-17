#include "ros/ros.h"
#include "stdio.h"
#include "globalpath/waypoint_data.h"

#include <iostream>

using namespace std;

float bearing_to_next = 0.0;

int main(int argc, char **argv){	
	ros::init(argc, argv, "test");
	
	ros::NodeHandle n;

	ros::Publisher pub3  = n.advertise<globalpath::waypoint_data>("/waypoint_data",1); 
	
	ros::Rate loop_rate(10);
	
	globalpath::waypoint_data waypointdata;

	cin>>bearing_to_next;

	while (ros::ok()){
		waypointdata.header.stamp = ros::Time::now();
		waypointdata.header.frame_id = "NEAD";
		waypointdata.angle = bearing_to_next;
		cout<<bearing_to_next<<endl;
		
		pub3.publish(waypointdata);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

