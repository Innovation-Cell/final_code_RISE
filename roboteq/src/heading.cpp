#include "ros/ros.h"
#include "roboteq/RoboteqDevice.h"
#include "roboteq/roboteq_msg.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"
#include "math.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "localization/vn100_msg.h"

using namespace std;

float v0 = 0.15;
float desired_heading = 10.0;
float vel = 0.15;

const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

const float Lr = 0.712;       
const float CIRC = 1.57; 
const float RED = 66.0;

const float correction_factor_ang = 0.55;
const float correction_factor_vel = 0.10/0.18;
const int turn_direction = 1; 
const int power_direction = 1; 

float w, v;

int status=0;
RoboteqDevice device;
 
const float PID_RESPONSE_SPEED = 1.0; 

const float P=1.1, I=0.1, D=0.7;
float err_new, err_old, errd, erri; 

float compute_error(float yaw){
	// cout<<desired_heading<<" "<<yaw<<endl;
	yaw -= desired_heading;

	if(yaw>180) yaw -= 360;
	else if (yaw<-180) yaw += 360;

	return yaw;
}

void motorcommand(){
	float pid = (PID_RESPONSE_SPEED*P*err_new + I*erri + D*errd);
	if (pid < (-20)) pid = -20;
	if (pid > 20) pid = 20;
	int _max = 1000; 
	device.GetConfig(_MXRPM, _max); 
	float max = _max;
	
	w = pid*correction_factor_ang*DEG_TO_RAD*(Lr/2)*(60/CIRC)*(float)turn_direction*(1000/max)*RED; 
	
	v0 = vel*(1-fabs(w)/1000);
			
	v = v0*correction_factor_vel*(60/CIRC)*(float)power_direction*(1000/max)*RED;                    		
	cout<<v<<" "<<w<<endl;		
	if( (((v+w)*power_direction)>2000) || (((v-w)*power_direction ) > 2000) ){ 
		device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
	}
	
	status = device.SetCommand(_G, 1, int(w));	
	status = device.SetCommand(_G, 2, int(v));
	
	v0 = 0;
}

void PID(const localization::vn100_msg::ConstPtr& info){
	std::cout<<"YO1"<<std::endl;	
	float yaw = info->linear.theta;
	
	float dt = 0.05;
	
	err_old = err_new; 
	err_new = compute_error(yaw);
		
	errd = (err_new-err_old)/dt;
	if (errd < (-10)) errd = -10;
	if (errd > 10) errd = 10;

	erri += (err_new + err_old)*dt/2;	
	if (erri < (-10)) erri = -10;
	if (erri > 10) erri = 10;
	
	motorcommand();
}

void deshead(const std_msgs::Float32::ConstPtr& msg_in){
	std::cout<<"YO2"<<std::endl;
	desired_heading = msg_in->data;
	cout<<desired_heading<<endl;
}

void no_obs(const std_msgs::String::ConstPtr& msg_data){
	string safety_message = msg_data->data;
}

int main(int argc, char* argv[]){	
	
	ros::init(argc, argv, "trajec_design");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
		
	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	
	status = device.Connect("/dev/ttyACM0");

	while (status != RQ_SUCCESS && ros::ok())
	{
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
	
	ros::Subscriber heading = n.subscribe("/vn100_monitor", 1, PID);
	ros::Subscriber desired_heading = n.subscribe("/desired_bearing", 1, deshead);
	ros::Subscriber lessDist=n.subscribe("/message", 1, no_obs);
	
	ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/husky/cmd_vel",1);
	
	cin>>vel;

	while(ros::ok()){
		ros::spinOnce();
		my_twist.linear.x=v;
		my_twist.angular.z=w;
		pub.publish(my_twist);
		//ROS_INFO("%f", my_twist.linear.x);
		//ROS_INFO("%f", my_twist.angular.z);
		loop_rate.sleep();
	}
	
	return 0;
}
