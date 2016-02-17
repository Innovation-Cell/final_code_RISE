// #include "ros/ros.h"
// #include "roboteq/RoboteqDevice.h"
// #include "roboteq/roboteq_msg.h"
// #include "roboteq/ErrorCodes.h"
// #include "roboteq/Constants.h"

// #include "nav_msgs/Odometry.h"
// #include "nav_msgs/OccupancyGrid.h"
// #include "std_msgs/Float32MultiArray.h"
// #include "std_msgs/Float64.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
// #include "std_msgs/Bool.h"
// #include "localization/vn100_msg.h"
// #include "tracking/customOccupancy.h"
// #include "nav_msgs/OccupancyGrid.h"

// #include "cmath"
// #include "string"

// using namespace std;

// std::vector<std::vector<float> > path_data;

// int width(100); 
// int height(100);
// float *map_data = NULL;

// int PATH_POINT_GAP(5), status(0);

// float  map_res(0.05);
// float MAX_VEL(0.2);
// float velocity(0);
// float err_old(0), err_new(0), errd(0), erri(0), pid(0);
// float curr_head(0);
// float final_head(0);
// float curr_xpos(0);
// float curr_ypos(0);
// float ang_vel(0);
// float yaw_f(0);

// const float RAD_TO_DEG(180/3.141592);
// const float DEG_TO_RAD(3.141592/180);

// const float Lr(0.712);       
// const float CIRC(1.57); 
// const float RED(66.0);

// const float correction_factor_ang = 0.55;
// const float correction_factor_vel = 0.20/0.30;
// const int turn_direction = -1; 
// const int power_direction = 1; 
 
// // const float PID_RESPONSE_SPEED(4.0/12.0); 
// const float P(2.0), I(1.0), D(0.1);

// bool odom_received(false), currenthead_received(false), map_received(false);
// bool newpath_required(true); 

// RoboteqDevice device;

// struct vel_time{
// 	float vel;
// 	float ang_vel;
// 	double time;
// };

// float yaw_err(float yaw_i){
// 	float yaw_e(0);
// 	yaw_e = yaw_i - final_head;
// 	if(yaw_e>180) yaw_e -= 360;
// 	else if (yaw_e<-180) yaw_e += 360;
// 	return yaw_e;
// }

// float calc_fangle(float dy, float dx){
// 	float angle(0);
// 	angle = atan2(dy , dx)*RAD_TO_DEG;

// 	if (angle >= -90 && angle <= 180) angle = 90 - angle;
// 	if (angle >= -180 && angle <= -90) angle = -(180 + (90 + angle));	

// 	return angle;
// }

// void motor_cmd(){
// 	float ang_vel, vel;
// 	int _max(1000); 
// 	device.GetConfig(_MXRPM, _max); 
// 	float max = _max;
	
// 	ang_vel = pid*correction_factor_ang*DEG_TO_RAD*(Lr/2.0)*(60.0/CIRC)*(float)turn_direction*(1000/max)*RED; 	
// 	vel = velocity*correction_factor_vel*(60.0/CIRC)*(float)power_direction*(1000/max)*RED;                  
// 	// cout<<ang_vel<<" "<<vel<<endl;  		
			
// 	if( (((vel + ang_vel)*power_direction)>2000) || (((vel - ang_vel)*power_direction) > 2000) ){ 
// 		device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
// 	}
	
// 	device.SetCommand(_G, 1, int(ang_vel));	
// 	device.SetCommand(_G, 2, int(vel));
// }

// void robot_cmd(vel_time *vt, int commands){
// 	std::cout<<"CMD"<<std::endl;
// 	double secs;
// 	float v, w, max;
// 	int i(0), _max;
// 	device.GetConfig(_MXRPM, _max);
// 	max = _max;

// 	while (i < commands){
// 		// w = (vt[i].ang_vel)*correction_factor_ang*DEG_TO_RAD*(Lr/2.0)*(60.0/CIRC)*(float)turn_direction*(1000/max)*RED; 
// 		// v = (vt[i].vel)*correction_factor_vel*(60.0/CIRC)*(float)power_direction*(1000/max)*RED;
// 		secs = ros::Time::now().toSec();
// 		while (ros::Time::now().toSec() <= (secs + vt[i].time)){
// 			// if( (((v+w)*power_direction)>2000) || (((v-w)*power_direction ) > 2000) ){ 
// 			// 	device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
// 			// } 
// 			// else{
// 			// 	device.SetCommand(_G, 1, int(w));	
// 			// 	device.SetCommand(_G, 2, int(v));
// 			// }
// 			velocity = vt[i].vel;
// 			final_head = vt[i].ang_vel;
// 			ros::spinOnce();
// 			motor_cmd();
// 		}
// 		i++;
// 	}
// 	device.SetCommand(_G, 1, 0); 
// 	device.SetCommand(_G, 2, 0);
// }

// void PID(const localization::vn100_msg::ConstPtr& info){
// 	float curr_head = info->linear.theta;
// 	currenthead_received = true;	
// 	float dt(0.05);
	
// 	err_old = err_new; 
// 	err_new = yaw_err(curr_head);
		
// 	errd = (err_new-err_old)/dt;
// 	if (errd < (-40)) errd = -40;
// 	if (errd > 40) errd = 40;

// 	erri += (err_new + err_old)*dt/2;	
// 	if (erri < (-40)) erri = -40;
// 	if (erri > 40) erri = 40;

// 	pid = (P*err_new + I*erri + D*errd);
// 	if (pid < (-20)) pid = -20;
// 	if (pid > 20) pid = 20;

// 	// cout<<"pid "<<pid<<endl;
// }

// bool check_occupancy_nextpos(float next_xpos, float next_ypos){
// 	bool check(false); 
// 	float dn(0), de(0);
// 	int xpos(0), ypos(0);
// 	dn = next_ypos - curr_ypos;
// 	de = next_xpos - curr_xpos;
// 	xpos = int((dn*cos(curr_head*DEG_TO_RAD) + de*sin(curr_head*DEG_TO_RAD))/map_res);
// 	ypos = (width/2-1) + int((dn*sin(curr_head*DEG_TO_RAD) - de*cos(curr_head*DEG_TO_RAD))/map_res);
// 	if (map_data[xpos*width + ypos] != 1) check = true; 
// 	return check;
// }

// void mapdata(const nav_msgs::OccupancyGrid::ConstPtr& map_in){
// 	width    = map_in->info.width;
// 	height   = map_in->info.height;
// 	map_res  = map_in->info.resolution;
// 	if (map_received == false){
// 		float* mdata = new float[width*height];
// 		map_data = mdata;
// 	}
// 	for (int i=0;i<(width*height);i++){
// 		map_data[i] = map_in->data[i];
// 	}
	
// 	map_received = true;
// }

// // void currenthead(const localization::vn100_msg::ConstPtr& theta_in){
// // 	curr_head = theta_in->linear.theta;
// // 	currenthead_received = true;
// // }

// void odom_combined(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_in){
// 	curr_ypos = odom_in->pose.pose.position.x;
// 	curr_xpos = odom_in->pose.pose.position.y;
// 	odom_received = true;
// }

// void get_path(const std_msgs::Float32MultiArray::ConstPtr& path_in){
// 	if (newpath_required = true){
// 		int pathsize(0);
// 		pathsize = path_in->data.size();
	
// 		if (path_data.size() != 0){
// 			for (int i=0;i<path_data.size();i++){
// 				path_data[i].clear();
// 			}
// 			path_data.clear();
// 		}

// 		path_data.resize(pathsize/2);
// 		for (int i=0;i<pathsize/2;i++){
// 			path_data[i].resize(2);
// 			path_data[i][0] = path_in->data[i];
// 			path_data[i][1]	= path_in->data[i+int(pathsize/2)];		
// 		}
// 		if (pathsize > 25) newpath_required = false;	
// 	}
// }

// int main(int argc, char* argv[]){	
	
// 	ros::init(argc, argv, "trajec_design_vel");
// 	ros::NodeHandle n;

// 	ros::Rate loop_rate(10);
		
// 	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
// 	ROS_INFO("Initializing...");
	
// 	status = device.Connect("/dev/ttyACM0");

// 	while (status != RQ_SUCCESS && ros::ok()){
// 		ROS_INFO("Error connecting to device: %d\n", status);
// 		ROS_INFO("Attempting server restart...");
// 		usleep(1000000);
// 		device.Disconnect();

// 		status = device.Connect("/dev/ttyACM0");
// 		if (status == 0) {
// 			ROS_INFO("Connection re-established...");
// 		}
// 	}
	
// 	int mode=0;
// 	device.GetConfig(_MXMD, mode);

// 	ROS_INFO("Operating mode: %d", mode);
// 	ros::Duration(1.0).sleep();
	
// 	device.SetConfig(_MXRPM, 1, 5000);
// 	device.SetConfig(_MXRPM, 2, 5000);

// 	geometry_msgs::Twist my_twist;
// 	std_msgs::Bool new_path;

// 	ros::Subscriber plannedpath = n.subscribe("/planned_path", 10, get_path);
// 	ros::Subscriber pose        = n.subscribe("/robot_pose_ekf/odom_combined", 10, odom_combined);
// 	ros::Subscriber sub         = n.subscribe("/scan/OccGrd", 10, mapdata);
// 	ros::Subscriber heading     = n.subscribe("/vn100_monitor", 10, PID);

// 	ros::Publisher pub1 = n.advertise<geometry_msgs::Twist>("husky/cmd_vel",1);
// 	ros::Publisher pub2 = n.advertise<std_msgs::Bool>("/newpath_required",1);

// 	float next_xpos(0), next_ypos(0), path_point_num(5);
// 	float dx(0), dy(0), dr(0), dt(0);//, yaw_i(0), dyaw(0), ang_v(0);
// 	int commands(0);
// 	float xpos, ypos;
// 	bool check_next(false);

// 	vel_time vel_t[100]; 

// 	while(ros::ok()){
// 		if (odom_received == true && newpath_required == false && currenthead_received == true){
// 			xpos = curr_xpos;
// 			ypos = curr_ypos;
// 			// yaw_i = curr_head;

// 			while (path_point_num <= 11){
// 				next_xpos = path_data[path_point_num][0];
// 				next_ypos = path_data[path_point_num][1];
// 				dx = next_xpos - xpos;
// 				dy = next_ypos - ypos;
// 				cout<<curr_xpos<<" "<<curr_ypos<<" "<<next_xpos<<" "<<next_ypos<<endl;
// 				cout<<dy<<" "<<dx<<endl;
// 				dr = pow((pow(dx,2)+pow(dy,2)),0.5);

// 				yaw_f = calc_fangle(dy, dx);
				
// 				// dyaw = yaw_f - yaw_i;
// 				dt = dr/MAX_VEL;
// 				// ang_v = dyaw/dt;
// 				vel_t[commands].vel = MAX_VEL;
// 				vel_t[commands].ang_vel = yaw_f;
// 				vel_t[commands].time = dt;
// 				std::cout<<dr<<" "<<dt<<std::endl;
// 				std::cout<<MAX_VEL<<" "<<yaw_f<<std::endl;
// 				//check_next = check_occupancy_nextpos(next_xpos, next_ypos);
// 				path_point_num += PATH_POINT_GAP;
// 				commands ++;
// 				std::cout<<commands<<std::endl;
// 				xpos = next_xpos;
// 				ypos = next_ypos;
// 				// yaw_i = yaw_f;	
// 			}
// 			robot_cmd(vel_t,commands);
			
// 			commands = 0;
// 			path_point_num = 2;
// 			std::cout<<"YO"<<std::endl;
// 			newpath_required = true;
// 		}

// 		// my_twist.linear.x = velocity;
// 		// my_twist.angular.z = pid;
// 		std::cout<<"YO1"<<std::endl;
// 		new_path.data = newpath_required;

// 		// pub1.publish(my_twist);
// 		pub2.publish(new_path);

// 		loop_rate.sleep();
// 		ros::spinOnce();
// 	}
// 	return 0;
// }

// #include "ros/ros.h"
// #include "roboteq/RoboteqDevice.h"
// #include "roboteq/roboteq_msg.h"
// #include "roboteq/ErrorCodes.h"
// #include "roboteq/Constants.h"

// #include "nav_msgs/Odometry.h"
// #include "nav_msgs/OccupancyGrid.h"
// #include "std_msgs/Float32MultiArray.h"
// #include "std_msgs/Float64.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
// #include "std_msgs/Bool.h"
// #include "localization/vn100_msg.h"
// #include "tracking/customOccupancy.h"
// #include "nav_msgs/OccupancyGrid.h"

// #include "cmath"
// #include "string"

// using namespace std;

// std::vector<std::vector<float> > path_data;

// int width(100); 
// int height(100);
// float *map_data = NULL;

// int PATH_POINT_GAP(10), status(0);

// float  map_res(0.05);
// float MAX_VEL(0.1);
// float velocity(0);
// float err_old(0), err_new(0), errd(0), erri(0), pid(0);
// float curr_head(0);
// float final_head(0);
// float curr_xpos(0);
// float curr_ypos(0);
// float ang_vel(0);
// float yaw_f(0);

// const float RAD_TO_DEG(180/3.141592);
// const float DEG_TO_RAD(3.141592/180);

// const float Lr(0.712);       
// const float CIRC(1.57); 
// const float RED(66.0);

// const float correction_factor_ang = 0.55;
// const float correction_factor_vel = 0.20/0.30;
// const int turn_direction = -1; 
// const int power_direction = 1; 
 
// // const float PID_RESPONSE_SPEED(4.0/12.0); 
// const float P(2.0), I(1.0), D(0.1);

// bool odom_received(false), currenthead_received(false), map_received(false);
// bool newpath_required(true); 

// RoboteqDevice device;

// struct vel_time{
// 	float vel;
// 	float ang_vel;
// 	double time;
// };

// // float yaw_err(float yaw_i){
// // 	float yaw_e(0);
// // 	yaw_e = yaw_i - final_head;
// // 	if(yaw_e>180) yaw_e -= 360;
// // 	else if (yaw_e<-180) yaw_e += 360;
// // 	return yaw_e;
// // }

// float calc_fangle(float dy, float dx){
// 	float angle(0);
// 	angle = atan2(dy , dx)*RAD_TO_DEG;

// 	if (angle >= -90 && angle <= 180) angle = 90 - angle;
// 	if (angle >= -180 && angle <= -90) angle = -(180 + (90 + angle));	

// 	return angle;
// }

// // void motor_cmd(){
// // 	float ang_vel, vel;
// // 	int _max(1000); 
// // 	device.GetConfig(_MXRPM, _max); 
// // 	float max = _max;
	
// // 	ang_vel = pid*correction_factor_ang*DEG_TO_RAD*(Lr/2.0)*(60.0/CIRC)*(float)turn_direction*(1000/max)*RED; 	
// // 	vel = velocity*(1-fabs(pid)/50)*correction_factor_vel*(60.0/CIRC)*(float)power_direction*(1000/max)*RED;                  
// // 	// cout<<ang_vel<<" "<<vel<<endl;  		
			
// // 	if( (((vel + ang_vel)*power_direction)>2000) || (((vel - ang_vel)*power_direction) > 2000) ){ 
// // 		device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
// // 	}
	
// // 	device.SetCommand(_G, 1, int(ang_vel));	
// // 	device.SetCommand(_G, 2, int(vel));
// // }

// void robot_cmd(vel_time *vt, int commands){
// 	std::cout<<"CMD"<<std::endl;
// 	double secs;
// 	float v, w, max;
// 	int i(0), _max(1000);
// 	device.GetConfig(_MXRPM, _max);
// 	max = _max;

// 	while (i < commands){
// 		w = (vt[i].ang_vel)*correction_factor_ang*DEG_TO_RAD*(Lr/2.0)*(60.0/CIRC)*(float)turn_direction*(1000/max)*RED; 
// 		v = (vt[i].vel)*correction_factor_vel*(60.0/CIRC)*(float)power_direction*(1000/max)*RED;
// 		secs = ros::Time::now().toSec();
// 		while (ros::Time::now().toSec() <= (secs + vt[i].time)){
// 			if( (((v+w)*power_direction)>2000) || (((v-w)*power_direction ) > 2000) ){ 
// 				device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
// 			} 
// 			else{
// 				device.SetCommand(_G, 1, int(w));	
// 				device.SetCommand(_G, 2, int(v));
// 			}
// 			// velocity = vt[i].vel;
// 			// final_head = vt[i].ang_vel;
// 			// ros::spinOnce();
// 			// motor_cmd();
// 		}
// 		i++;
// 	}
// 	device.SetCommand(_G, 1, 0); 
// 	device.SetCommand(_G, 2, 0);
// }

// // void PID(const localization::vn100_msg::ConstPtr& info){
// // 	float curr_head = info->linear.theta;
// // 	currenthead_received = true;	
// // 	float dt(0.05);
	
// // 	err_old = err_new; 
// // 	err_new = yaw_err(curr_head);
		
// // 	errd = (err_new-err_old)/dt;
// // 	if (errd < (-40)) errd = -40;
// // 	if (errd > 40) errd = 40;

// // 	erri += (err_new + err_old)*dt/2;	
// // 	if (erri < (-40)) erri = -40;
// // 	if (erri > 40) erri = 40;

// // 	pid = (P*err_new + I*erri + D*errd);
// // 	if (pid < (-20)) pid = -20;
// // 	if (pid > 20) pid = 20;

// // 	// cout<<"pid "<<pid<<endl;
// // }

// bool check_occupancy_nextpos(float next_xpos, float next_ypos){
// 	bool check(false); 
// 	float dn(0), de(0);
// 	int xpos(0), ypos(0);
// 	dn = next_ypos - curr_ypos;
// 	de = next_xpos - curr_xpos;
// 	xpos = int((dn*cos(curr_head*DEG_TO_RAD) + de*sin(curr_head*DEG_TO_RAD))/map_res);
// 	ypos = (width/2-1) + int((dn*sin(curr_head*DEG_TO_RAD) - de*cos(curr_head*DEG_TO_RAD))/map_res);
// 	if (map_data[xpos*width + ypos] != 1) check = true; 
// 	return check;
// }

// void mapdata(const nav_msgs::OccupancyGrid::ConstPtr& map_in){
// 	width    = map_in->info.width;
// 	height   = map_in->info.height;
// 	map_res  = map_in->info.resolution;
// 	if (map_received == false){
// 		float* mdata = new float[width*height];
// 		map_data = mdata;
// 	}
// 	for (int i=0;i<(width*height);i++){
// 		map_data[i] = map_in->data[i];
// 	}
	
// 	map_received = true;
// }

// void currenthead(const localization::vn100_msg::ConstPtr& theta_in){
// 	curr_head = theta_in->linear.theta;
// 	currenthead_received = true;
// }

// void odom_combined(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_in){
// 	curr_ypos = odom_in->pose.pose.position.x;
// 	curr_xpos = odom_in->pose.pose.position.y;
// 	odom_received = true;
// }

// void get_path(const std_msgs::Float32MultiArray::ConstPtr& path_in){
// 	if (newpath_required = true){
// 		int pathsize(0);
// 		pathsize = path_in->data.size();
	
// 		if (path_data.size() != 0){
// 			for (int i=0;i<path_data.size();i++){
// 				path_data[i].clear();
// 			}
// 			path_data.clear();
// 		}

// 		path_data.resize(pathsize/2);
	
// 		for (int i=0;i<pathsize/2;i++){
// 			path_data[i].resize(2);
// 			path_data[i][0] = path_in->data[i];
// 			path_data[i][1]	= path_in->data[i+int(pathsize/2)];		
// 		}

// 		if (pathsize > 25) newpath_required = false;	
// 	}
// }

// int main(int argc, char* argv[]){	
	
// 	ros::init(argc, argv, "trajec_design_vel");
// 	ros::NodeHandle n;

// 	ros::Rate loop_rate(10);
		
// 	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
// 	ROS_INFO("Initializing...");
	
// 	status = device.Connect("/dev/ttyACM0");

// 	while (status != RQ_SUCCESS && ros::ok()){
// 		ROS_INFO("Error connecting to device: %d\n", status);
// 		ROS_INFO("Attempting server restart...");
// 		usleep(1000000);
// 		device.Disconnect();

// 		status = device.Connect("/dev/ttyACM0");
// 		if (status == 0) {
// 			ROS_INFO("Connection re-established...");
// 		}
// 	}
	
// 	int mode=0;
// 	device.GetConfig(_MXMD, mode);

// 	ROS_INFO("Operating mode: %d", mode);
// 	ros::Duration(1.0).sleep();
	
// 	device.SetConfig(_MXRPM, 1, 5000);
// 	device.SetConfig(_MXRPM, 2, 5000);

// 	geometry_msgs::Twist my_twist;
// 	std_msgs::Bool new_path;

// 	ros::Subscriber plannedpath = n.subscribe("/planned_path", 10, get_path);
// 	ros::Subscriber pose        = n.subscribe("/robot_pose_ekf/odom_combined", 10, odom_combined);
// 	ros::Subscriber sub         = n.subscribe("/scan/OccGrd", 10, mapdata);
// 	ros::Subscriber heading     = n.subscribe("/vn100_monitor", 10, currenthead);

// 	ros::Publisher pub1 = n.advertise<geometry_msgs::Twist>("husky/cmd_vel",1);
// 	ros::Publisher pub2 = n.advertise<std_msgs::Bool>("/newpath_required",1);

// 	float next_xpos(0), next_ypos(0), path_point_num(10);
// 	float dx(0), dy(0), dr(0), dt(0), yaw_i(0), dyaw(0), ang_v(0);
// 	int commands(0);
// 	float xpos, ypos;
// 	bool check_next(false);

// 	vel_time vel_t[100]; 

// 	while(ros::ok()){
// 		if (odom_received == true && newpath_required == false && currenthead_received == true){
// 			xpos = curr_xpos;
// 			ypos = curr_ypos;
// 			// yaw_i = curr_head;

// 			while (path_point_num <= 21){
// 				next_xpos = path_data[path_point_num][0];
// 				next_ypos = path_data[path_point_num][1];
// 				dx = next_xpos - xpos;
// 				dy = next_ypos - ypos;

// 				cout<<curr_xpos<<" "<<curr_ypos<<" "<<next_xpos<<" "<<next_ypos<<endl;
// 				cout<<dy<<" "<<dx<<endl;

// 				dr = pow((pow(dx,2)+pow(dy,2)),0.5);
// 				dy = path_data[path_point_num + PATH_POINT_GAP][1] - next_ypos;
// 				dx = path_data[path_point_num + PATH_POINT_GAP][0] - next_xpos;
// 				yaw_f = calc_fangle(dy, dx);
				
// 				dyaw = yaw_f - yaw_i;
// 				dt = dr/MAX_VEL;
// 				ang_v = dyaw/dt;

// 				vel_t[commands].vel = MAX_VEL;
// 				vel_t[commands].ang_vel = ang_v;
// 				vel_t[commands].time = dt;
// 				std::cout<<dr<<" "<<dt<<std::endl;
// 				std::cout<<ang_v<<" "<<yaw_f<<std::endl;
// 				//check_next = check_occupancy_nextpos(next_xpos, next_ypos);

// 				path_point_num += PATH_POINT_GAP;
// 				commands ++;
// 				std::cout<<commands<<std::endl;
// 				xpos = next_xpos;
// 				ypos = next_ypos;

// 				dx = next_xpos - xpos;
// 				dy = next_ypos - ypos;
// 				yaw_i = calc_fangle(dy, dx);	
// 			}
// 			robot_cmd(vel_t,commands);
			
// 			commands = 0;
// 			path_point_num = 2;
// 			std::cout<<"YO"<<std::endl;
// 			newpath_required = true;
// 		}

// 		// my_twist.linear.x = velocity;
// 		// my_twist.angular.z = pid;
// 		std::cout<<"YO1"<<std::endl;
// 		new_path.data = newpath_required;

// 		// pub1.publish(my_twist);
// 		pub2.publish(new_path);

// 		loop_rate.sleep();
// 		ros::spinOnce();
// 	}
// 	return 0;
// }



#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Bool.h"
#include "localization/vn100_msg.h"
#include "tracking/customOccupancy.h"
#include "nav_msgs/OccupancyGrid.h"

#include "cmath"
#include "string"

using namespace std;

std::vector<std::vector<float> > path_data;

int width(100); 
int height(100);
float *map_data = NULL;

int PATH_POINT_GAP(5), status(0), path_point_num(5);

float  map_res(0.05);
float curr_head(0);
float curr_xpos(0);
float curr_ypos(0);

const float RAD_TO_DEG(180/3.141592);
const float DEG_TO_RAD(3.141592/180);

bool odom_received(false), currenthead_received(false), map_received(false);
bool newpath_required(true); 

float calc_fangle(float dy, float dx){
	float angle(0);
	angle = atan2(dy , dx)*RAD_TO_DEG;

	if (angle >= -90 && angle <= 180) angle = 90 - angle;
	if (angle >= -180 && angle <= -90) angle = -(180 + (90 + angle));	

	return angle;
}

void currenthead(const localization::vn100_msg::ConstPtr& info){
	// std::cout<<"YO1"<<std::endl;
	float curr_head = info->linear.theta;
	currenthead_received = true;	
}

bool check_occupancy_nextpos(float next_xpos, float next_ypos){
	bool check(false); 
	float dn(0), de(0);
	int xpos(0), ypos(0);
	dn = next_ypos - curr_ypos;
	de = next_xpos - curr_xpos;
	xpos = int((dn*cos(curr_head*DEG_TO_RAD) + de*sin(curr_head*DEG_TO_RAD))/map_res);
	ypos = (width/2-1) + int((dn*sin(curr_head*DEG_TO_RAD) - de*cos(curr_head*DEG_TO_RAD))/map_res);
	if (map_data[xpos*width + ypos] != 1) check = true; 
	return check;
}

void mapdata(const nav_msgs::OccupancyGrid::ConstPtr& map_in){
	width    = map_in->info.width;
	height   = map_in->info.height;
	map_res  = map_in->info.resolution;
	if (map_received == false){
		float* mdata = new float[width*height];
		map_data = mdata;
	}
	for (int i=0;i<(width*height);i++){
		map_data[i] = map_in->data[i];
	}
	
	map_received = true;
}

void odom_combined(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_in){
	curr_ypos = odom_in->pose.pose.position.x;
	curr_xpos = odom_in->pose.pose.position.y;
	odom_received = true;
}

// void odom_combined(const nav_msgs::Odometry::ConstPtr& odom_in1){
// 	std::cout<<"YO1"<<std::endl;
// 	curr_ypos = odom_in1->pose.pose.position.x;
// 	curr_xpos = odom_in1->pose.pose.position.y;
// 	odom_received = true;
// }

void get_path(const std_msgs::Float32MultiArray::ConstPtr& path_in){
	if (newpath_required = true){
		int pathsize(0);
		pathsize = path_in->data.size();
	
		if (path_data.size() != 0){
			for (int i=0;i<path_data.size();i++){
				path_data[i].clear();
			}
			path_data.clear();
		}

		path_data.resize(pathsize/2);
		for (int i=0;i<pathsize/2;i++){
			path_data[i].resize(2);
			path_data[i][0] = path_in->data[i];
			path_data[i][1]	= path_in->data[i+int(pathsize/2)];		
		}

		if (pathsize > 2){
			if (pathsize > 10){
				path_point_num = 9;
			}
			else path_point_num = 4;
		}
		else path_point_num = 1;

		newpath_required = false;	
	}
}

int main(int argc, char* argv[]){	
	
	ros::init(argc, argv, "trajec_design_vel");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	geometry_msgs::Twist my_twist;
	std_msgs::Bool new_path;

	ros::Subscriber plannedpath = n.subscribe("/planned_path", 10, get_path);
	ros::Subscriber pose        = n.subscribe("/robot_pose_ekf/odom_combined", 10, odom_combined);
	ros::Subscriber sub         = n.subscribe("/scan/OccGrd", 10, mapdata);
	ros::Subscriber heading     = n.subscribe("/vn100_monitor", 10, currenthead);

	ros::Publisher pub1 = n.advertise<std_msgs::Float32>("/desired_bearing",1);
	ros::Publisher pub2 = n.advertise<std_msgs::Bool>("/newpath_required",1);

	float next_xpos(0), next_ypos(0);
	float dx(0), dy(0), dr(0), dt(0);

	float xpos, ypos;
	bool check_next(false);

	std_msgs::Float32 final_head; 

	while(ros::ok()){
		if (odom_received == true && newpath_required == false && currenthead_received == true){
		// std::cout<<"YO1"<<std::endl;
		xpos = curr_xpos;
		ypos = curr_ypos;

		next_xpos = path_data[path_point_num][0];
		next_ypos = path_data[path_point_num][1];

		dx = float(next_xpos) - xpos;
		dy = float(next_ypos) - ypos;

		final_head.data = calc_fangle(dy, dx); 
		cout<<final_head.data<<endl;

		newpath_required = true;

		new_path.data = newpath_required;

		pub2.publish(new_path);
		pub1.publish(final_head);

		}
	loop_rate.sleep();
	ros::spinOnce();
	}
	return 0;
}
