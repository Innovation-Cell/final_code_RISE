#include "ros/ros.h"
#include "tf/tf.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"

#include <iostream>
#include <stdio.h>
#include <math.h>

#include "vectornav.h"

#include "localization/vn100_msg.h"
#include "localization/lp.h"



using namespace std;

// const char* const COM_PORT = "/dev/ttyUSB0";
// const int BAUD_RATE = 115200;
// const float INV_CUTOFF = 1/20.0;
const float RAD_TO_DEG = 180/3.141592;
const double tare = 103.4;

static sensor_msgs::Imu output;

float x_imu = 0, y_imu = 0, z_imu = 0, w_imu = 0, wx = 0, wy = 0, wz = 0, ax = 0, ay = 0, az = 0;
double tarefix_yaw = 0, tarefix_pitch = 0, tarefix_roll = 0;

// void low_pass(localization::lp &unfiltered){
// 	float dt = unfiltered.stamp.toSec() - output.header.stamp.toSec();

// 	output.header.stamp = unfiltered.stamp;

// 	float weight = (dt)/(dt + INV_CUTOFF);

// 	unfiltered.av_x *= RAD_TO_DEG; unfiltered.av_y *= RAD_TO_DEG; unfiltered.av_z *= RAD_TO_DEG; 
	
// 	output.linear_acceleration.x = (weight*unfiltered.x) + ((1-weight)*output.linear_acceleration.x);
// 	output.linear_acceleration.y = (weight*unfiltered.y) + ((1-weight)*output.linear_acceleration.y);
// 	output.linear_acceleration.z = (weight*unfiltered.z) + ((1-weight)*output.linear_acceleration.z);
// 	output.angular_velocity.x = (weight*unfiltered.av_x) + ((1-weight)*output.angular_velocity.x);
// 	output.angular_velocity.y = (weight*unfiltered.av_y) + ((1-weight)*output.angular_velocity.y);
// 	output.angular_velocity.z = (weight*unfiltered.av_z) + ((1-weight)*output.angular_velocity.z);

// 	if(output.linear_acceleration.x<0.3&&output.linear_acceleration.x>(-0.3)) output.linear_acceleration.x = 0;
// 	if(output.linear_acceleration.y<0.3&&output.linear_acceleration.y>(-0.3)) output.linear_acceleration.y = 0;
// 	if(output.linear_acceleration.z<0.3&&output.linear_acceleration.z>(-0.3)) output.linear_acceleration.z = 0;	
// 	if(output.angular_velocity.x<0.3&&output.angular_velocity.x>(-0.3)) output.angular_velocity.x = 0;
// 	if(output.angular_velocity.y<0.3&&output.angular_velocity.y>(-0.3)) output.angular_velocity.y = 0;
// 	if(output.angular_velocity.z<0.3&&output.angular_velocity.z>(-0.3)) output.angular_velocity.z = 0;

// }	

void spartonCallback(const sensor_msgs::Imu::ConstPtr& info)
{
	x_imu = info->orientation.x;
	y_imu = info->orientation.y;
	z_imu = info->orientation.z;
	w_imu = info->orientation.w;

	wx = info->angular_velocity.x;
	wy = info->angular_velocity.y;
	wz = info->angular_velocity.z;

	ax = info->linear_acceleration.x;
	ay = info->linear_acceleration.y;
	az = info->linear_acceleration.z;
}



int main(int argc, char **argv)
{
	///Variable Declerations
	// Vn100 vn100;
	// VnYpr attitude;
	// VnVector3 bodyAcceleration, angularRate;
	// VnVector3 angularRateVariance, magneticVariance, accelerationVariance;
	// double angularWalkVariance;
	
	///ROS Node Initialization
	ros::init(argc, argv, "Sparton_Interface"); 
	ros::NodeHandle imu;

	ros::Publisher imuDataPublisher = imu.advertise<sensor_msgs::Imu>("/imu_data", 1);
	ros::Publisher infer = imu.advertise<localization::vn100_msg>("/vn100_monitor", 1);

	ros::Subscriber spartonMonitor = imu.subscribe("/imu/data", 1, spartonCallback);

	///Connection to Vectornav
	// int connect = 0;
	// ROS_INFO("\n\n--- Vectornav IMU Request Gateway Server ---\n");
	// ROS_INFO("Initializing...");
	// connect = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
	// ROS_INFO("Connected ...");
	// while(connect != 0){
	// 	ROS_ERROR("Not Connected ...");
	// 	usleep(999999);
 // 	}
	// ROS_INFO("Server initialized...");
		
	///Publishing Loops
	ros::Rate loop_rate(20);
	
	localization::lp unfiltered;
	localization::vn100_msg info;
	output.header.frame_id = "imu";

	while (ros::ok()) {
		//Getting Data//
		// if(vn100.isConnected){	
		// 	vn100_getYawPitchRollTrueInertialAccelerationAngularRate(&vn100, &attitude, &bodyAcceleration, &angularRate);
		// } else ROS_ERROR("Connection Error...");

		// if(vn100.isConnected){	
		// 	vn100_getFilterMeasurementVarianceParameters(&vn100, &angularWalkVariance, &angularRateVariance, &magneticVariance, &accelerationVariance);
		// } else ROS_ERROR("Connection Error...");

		//Filling in Data//
			//Data for Low Pass Filter
				//Header data
				// unfiltered.stamp = ros::Time::now();
				// //IMU data
				// unfiltered.x = bodyAcceleration.c0; unfiltered.y = bodyAcceleration.c1; unfiltered.z = bodyAcceleration.c2;
				// unfiltered.av_x = angularRate.c0; unfiltered.av_y = angularRate.c1; unfiltered.av_z = angularRate.c2;
				// low_pass(unfiltered);

			
			//Data for Publishing
				//Header data
				output.header.seq++;
				output.header.stamp = ros::Time::now();
				//Orientation
				tf::Quaternion rotation(x_imu,y_imu,z_imu,w_imu);
				tf::Matrix3x3 m(rotation);
				// rotation.setRPY(attitude.roll*(1/RAD_TO_DEG), attitude.pitch*(1/RAD_TO_DEG), ((attitude.yaw) + tare)*(1/RAD_TO_DEG));
				m.getRPY(tarefix_roll, tarefix_pitch, tarefix_yaw);
				// cout<<attitude.roll<<" "<<attitude.pitch<<" "<<attitude.yaw + tare<<endl;

				tarefix_roll *= RAD_TO_DEG;
				tarefix_pitch *= RAD_TO_DEG;
				tarefix_yaw *= RAD_TO_DEG;

				tarefix_yaw = -tarefix_yaw;
				tarefix_yaw += tare;

				if(tare > 0){
					if (tarefix_yaw > 180){
						tarefix_yaw =  (-180 + tare) - (tare - (tarefix_yaw - 180)); 
					}
				}

				if(tare < 0){
					if (tarefix_yaw < -180){
						tarefix_yaw = (180 + tare) + (-tare + 180 + tarefix_yaw); 
					}
				}
				
				//cout << tarefix_roll << " " << tarefix_pitch << " " << tarefix_yaw << endl;

				tf::Quaternion rotation_final;
				rotation_final.setRPY(tarefix_roll*(1/RAD_TO_DEG), tarefix_pitch*(1/RAD_TO_DEG), tarefix_yaw*(1/RAD_TO_DEG));

				output.orientation.x = (double) rotation_final.x();
				output.orientation.y = (double) rotation_final.y();
				output.orientation.z = (double) rotation_final.z();
				output.orientation.w = (double) rotation_final.w();

				output.linear_acceleration.x = ax;
				output.linear_acceleration.y = ay;
				output.linear_acceleration.z = az;

				output.angular_velocity.x = wx;
				output.angular_velocity.y = wy;
				output.angular_velocity.z = wz;

				for (int i=0; i<9; i++){
					output.orientation_covariance[i] = 0;
				}
				output.orientation_covariance[0] = 0.000001;
				output.orientation_covariance[4] = 0.000001;
				output.orientation_covariance[8] = 0.000001;
				//Angular Velocity Variance
				for(int i=0; i<9; i++){
					output.angular_velocity_covariance[i] = 0;
				}
				output.angular_velocity_covariance[0] = 0.000001;
				output.angular_velocity_covariance[4] = 0.000001;
				output.angular_velocity_covariance[8] = 0.000001;
				//Linear Acceleration Variance
				for(int i=0; i<9; i++){
					output.linear_acceleration_covariance[i] = 0;
				}
				output.linear_acceleration_covariance[0] = 0.000001;
				output.linear_acceleration_covariance[4] = 0.000001;
				output.linear_acceleration_covariance[8] = 0.000001;


				// //For vn100_monitor (i.e. for running the vehicle along a particular heading)
				info.header.stamp = output.header.stamp; info.header.seq++;
				info.linear.x = output.linear_acceleration.x; info.linear.y = output.linear_acceleration.y;
				info.linear.theta = tarefix_yaw;
				info.angular_velocity = output.angular_velocity.z;
		
		//double headingDebugging = tf::getYaw(output.orientation);
		//ROS_INFO("Heading sent: %f", headingDebugging);
		
		//Other Stuff//
		imuDataPublisher.publish(output);
		infer.publish(info);
		ros::spinOnce();

		loop_rate.sleep();

	}

	return 0;

}

