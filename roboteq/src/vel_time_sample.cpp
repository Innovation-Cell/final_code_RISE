#include "ros/ros.h"
#include "roboteq/RoboteqDevice.h"
#include "roboteq/roboteq_msg.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

using namespace std;

float v0 = 0;
float vel = 0.2;

const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

const float Lr = 0.712;       
const float CIRC = 1.57; 
const float RED = 66.0;

const float correction_factor_ang = 0.55;
const float correction_factor_vel = 0.20/0.18;
const int turn_direction = -1; 
const int power_direction = 1; 

float w, v;

int status=0;
RoboteqDevice device;

struct vel_time{
	float vel;
	float ang_vel;
	double time;
};

void motorcommand(vel_time *vt, int commands){
	cout<<"YO2"<<endl;
	double secs;
	float v, w, max;
	int i(0), _max(1000); 
	device.GetConfig(_MXRPM, _max);
	max = _max;

	while (i < commands){
		cout<<"YO3"<<endl;
		w = (vt[i].ang_vel)*correction_factor_ang*DEG_TO_RAD*(Lr/2.0)*(60.0/CIRC)*(float)turn_direction*(1000/max)*RED; 
		v = (vt[i].vel)*correction_factor_vel*(60.0/CIRC)*(float)power_direction*(1000/max)*RED;
		secs = ros::Time::now().toSec();
		cout<<"YO4"<<endl;
		while (ros::Time::now().toSec() <= secs + vt[i].time){
			if( (((v+w)*power_direction)>2000) || (((v-w)*power_direction ) > 2000) ){ 
				device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
			} 
			else{
				device.SetCommand(_G, 1, int(w));	
				device.SetCommand(_G, 2, int(v));
			}
		}
		i++;
	}
	device.SetCommand(_G, 1, 0); 
	device.SetCommand(_G, 2, 0);
}

int main(int argc, char* argv[]){	
	
	ros::init(argc, argv, "trajec_design");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
		
	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	
	status = device.Connect("/dev/ttyACM0");

	while (status != RQ_SUCCESS && ros::ok()){
		ROS_INFO("Error connecting to device: %d\n", status);
		ROS_INFO("Attempting server restart...");
		usleep(1000000);
		device.Disconnect();

		status = device.Connect("/dev/ttyACM0");
		if (status == 0) {
			ROS_INFO("Connection re-established...");
		}
	}
	
	int mode=0;
	device.GetConfig(_MXMD, mode);
	ROS_INFO("Operating mode: %d", mode);
	ros::Duration(1.0).sleep();
	
	geometry_msgs::Twist my_twist;
	
	device.SetConfig(_MXRPM, 1, 5000);
	device.SetConfig(_MXRPM, 2, 5000);
	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/husky/cmd_vel",1);

	int commands = 5;
	vel_time vt[commands];
	
	// float velocity[] = { 0.1, 0, 0.1, 0, 0.1, 0, 0.1, 0 };
	// float angular_velocity[] = { 0, 90, 0, 90, 0, 90, 0, 90 };
	// double t[] = { 5.0, 1.0, 5.0, 1.0, 5.0, 1.0, 5.0, 1.0 };

	float velocity[] = { 0.1, 0.1, 0.1, 0.1, 0.1 };
	float angular_velocity[] = { 60.0, -60.0, 60.0, -60.0 ,60.0 };
	double t[] = { 2.0, 2.0, 2.0, 2.0, 2.0 };

	for (int i=0;i<commands;i++){
		vt[i].vel = velocity[i];
		vt[i].ang_vel = angular_velocity[i];
		vt[i].time = t[i];
	}

	motorcommand(vt,commands);

	while(ros::ok()){
		cout<<"YO"<<endl;
		// device.SetCommand(_G, 1, int(100));
		cout<<"YO1"<<endl;
		ros::spinOnce();
		my_twist.linear.x=v;
		my_twist.angular.z=w;
		pub.publish(my_twist);
		loop_rate.sleep();
	}
	
	return 0;
}
