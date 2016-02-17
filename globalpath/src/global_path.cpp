#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdio.h"
#include "vectornav/vn200_msg.h"
#include "geodesy/utm.h"
#include "globalpath/northing_easting.h"
#include "globalpath/waypoint_data.h"
#include "std_msgs/Float32.h"
#include "gps_common/conversions.h"
#include "sensor_msgs/NavSatFix.h"

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;

float waypoint_achievement_accuracy = 5.0;

double UTMnorthing(0);
double UTMeasting(0);

float origin_lat(19.13278216);
float origin_lon(72.9157759);
double origin_xpos(0);
double origin_ypos(0);

string zone;
string data;

vector<vector<double> > coordinates;
vector<vector<double> > northingeasting;

int nofcoors(0);
int current_waypoint(0);
int next_waypoint(0);

bool process_completion(false);
bool mod_start(false);
bool destination_reached(false);
bool to_first_waypoint(true);

float current_lat(0);
float current_lon(0);

float calc_angle(float dy, float dx){
	float angle(0);
	angle = atan2(dy , dx)*RAD_TO_DEG;

	if (angle >= -90 && angle <= 180) angle = 90 - angle;
	if (angle >= -180 && angle <= -90) angle = -(180 + (90 + angle));	

	return angle;
}

double strtof(string str){      	
	int len=str.length(), pos=str.find ('.');
	double num = 0;
	
	for (int i=0;i<len;i++){
		int deci=str[i]-'0';
		if (i < pos) num += deci*(pow(10,(pos-i-1)));
		if (i > pos) num += deci*(pow(10,(pos-i)));
	}
	return num;
}
	
void strtofcoors(string strcoors){
	if (process_completion == false){
		int j=0;
		int len = strcoors.length();
	
		for (int i=0 ;i<len;i++){
			if (strcoors[i] == '(' ) j++;
		}
	
		nofcoors = j;
		northingeasting.resize(nofcoors);
	
		coordinates.resize(j);
	
		for (int k=0;k<j;k++){
			int l=0;
			int posparenstart = strcoors.find ('(');
			int poscomma = strcoors.find (',');
			int posparenstop = strcoors.find (')');
			coordinates[k].resize(2);
			coordinates[k][l] = strtof(strcoors.substr (posparenstart+1,poscomma-posparenstart-1));
			coordinates[k][l+1] = strtof(strcoors.substr (poscomma+2,posparenstop-poscomma-2));
			strcoors.erase(posparenstart,posparenstop-posparenstart+3);
		}
	
		process_completion = true;	
	} 
}

void data_svr_con(const std_msgs::String::ConstPtr& msg){
	data = msg->data.c_str(); 
	strtofcoors(data);
}

void pos_assign(const vectornav::vn200_msg::ConstPtr& info){
	current_lat = info->LLA.latitude;	
	current_lon = info->LLA.longitude;

	if (mod_start == false){
		origin_lat = current_lat;
		origin_lon = current_lon;
		
		gps_common::LLtoUTM(origin_lat,origin_lon,UTMnorthing,UTMeasting,zone);
		
		origin_xpos = UTMeasting;
		origin_ypos = UTMnorthing;
		cout<<"origin lat, lon, xpos, ypos :"<<origin_lat<<" "<<origin_lon<<" "<<origin_xpos<<" "<<origin_ypos<<endl;
	}
	mod_start = true;
}

int main(int argc, char **argv){	
	ros::init(argc, argv, "northingeasting");
	
	ros::NodeHandle n;

	ros::Subscriber sub  = n.subscribe("/gps_pnts", 1, data_svr_con);
	ros::Subscriber sub2 = n.subscribe("/gps/fix", 1, pos_assign);

	ros::Publisher pub2  = n.advertise<globalpath::northing_easting>("/global_north_east",1); 
	ros::Publisher pub3  = n.advertise<globalpath::waypoint_data>("/waypoint_data",1); 
	
	ros::Rate loop_rate(10);
	
	globalpath::northing_easting northing_easting;
	globalpath::waypoint_data waypointdata;

	double dist_x(0), dist_y(0), dist_r(0), bearing_to_next(0), distance_to_next(0);
	double UTMnorthing2(0), UTMeasting2(0);
	
	while (ros::ok()){	
		if (process_completion == true && destination_reached == false && mod_start == true){
			northing_easting.header.stamp = ros::Time::now();
			northing_easting.header.frame_id = "NE";
			northing_easting.northing.clear();
			northing_easting.easting.clear();
			
			for(int i=0; i<nofcoors; i++){
				northingeasting[i].resize(2);
				gps_common::LLtoUTM(coordinates[i][0],coordinates[i][1],UTMnorthing,UTMeasting,zone);
				northingeasting[i][0] = UTMnorthing; 
				northingeasting[i][1] = UTMeasting;
				northing_easting.northing.push_back(northingeasting[i][0]);
				northing_easting.easting.push_back(northingeasting[i][1]); 
			}

			gps_common::LLtoUTM(current_lat,current_lon,UTMnorthing,UTMeasting,zone);
			dist_x = northingeasting[next_waypoint][1] - UTMeasting; 
			dist_y = northingeasting[next_waypoint][0] - UTMnorthing; 			
 			dist_r = pow((pow(dist_x,2)+pow(dist_y,2)),0.5);

			if (dist_r < waypoint_achievement_accuracy){
				if (next_waypoint == (nofcoors-1)){
					destination_reached = true;
					cout<<"The vehicle is at the destination"<<endl;
				}
				else next_waypoint += 1;
			}

			if (destination_reached == false){
				gps_common::LLtoUTM(coordinates[next_waypoint][0],coordinates[next_waypoint][1],UTMnorthing2,UTMeasting2,zone);
				gps_common::LLtoUTM(current_lat,current_lon,UTMnorthing,UTMeasting,zone);

				bearing_to_next = calc_angle((UTMnorthing2-UTMnorthing), (UTMeasting2-UTMeasting));
				distance_to_next = dist_r;

				gps_common::LLtoUTM(coordinates[next_waypoint][0],coordinates[next_waypoint][1],UTMnorthing,UTMeasting,zone);

				waypointdata.header.stamp = ros::Time::now();
				waypointdata.header.frame_id = "NEAD";
				waypointdata.northing = UTMnorthing - origin_ypos;
				waypointdata.easting = UTMeasting - origin_xpos;
				waypointdata.angle = bearing_to_next;
				waypointdata.distance = distance_to_next;
				cout<<"angle, distance :"<<bearing_to_next<<" "<<distance_to_next<<endl;
			}		

			pub2.publish(northing_easting);
			pub3.publish(waypointdata);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

