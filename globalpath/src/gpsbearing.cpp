#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "stdio.h"
#include <string>
#include <vector>
#include <cmath>
#include "vectornav/vn200_msg.h"
#include "geodesy/utm.h"
#include "globalpath/bearing_distance.h"
#include "std_msgs/Float32.h"

using namespace std ;

vector<vector<double> > coordinates;
vector<vector<double> > bearingDistance;
vector<double> distance_viewer;

string data;

const float pi = 3.14159265;
const float earthradius = 6371000.000;     //average earth radius
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;

int nofcoors = 0;
int process_completion = 0;

float current_lat = 19.13278216;
float current_lon = 72.9157759;
float goal_distance = 0;

double strtof(string str){          //function to convert data from string to float 	
	int len=str.length(), pos=str.find ('.');
	double num = 0;
	
	for (int i=0;i<len;i++){
		int deci=str[i]-'0';
		if (i < pos) num += deci*(pow(10,(pos-i-1)));
		if (i > pos) num += deci*(pow(10,(pos-i)));
	}
	return num;
}

double Distance(double latitude1, double longitude1, double latitude2, double longitude2){     //function to calculate the great circle distance b/w gps coordinates 
	double lat1, latrad1, lat2, latrad2, lon1, lon2, difflatrad, difflonrad, a, c, distance;
	lat1 = latitude1;
	latrad1 = DEG_TO_RAD*lat1;
	
	lat2 = latitude2;
	latrad2 = DEG_TO_RAD*lat2;

	lon1 = longitude1;
	lon2 = longitude2;
	
	difflatrad = DEG_TO_RAD*(lat2-lat1);
	difflonrad = DEG_TO_RAD*(lon2-lon1);
	
    a = pow((sin (difflatrad/2)),2) + (cos (latrad1))*(cos (latrad2))*(pow(sin(difflonrad/2),2));
    double d = pow(a,(0.5));
    double b = pow((1+(-1)*a),0.5);
    c=(2.0)*(atan2 (d,b));
    distance = earthradius*c;

    return distance;
}

double Bearing(double latitude1, double longitude1, double latitude2, double longitude2){   //function to calculate bearing wrt north   
	double lat1, lat2, lon1, lon2, difflon, bear, bearing, a, b; 
	
	lat1 = DEG_TO_RAD*latitude1;
	lat2 = DEG_TO_RAD*latitude2;
	lon1 = DEG_TO_RAD*longitude1;
	lon2 = DEG_TO_RAD*longitude2;
	
	difflon = DEG_TO_RAD*(longitude2+(-1)*longitude1);
	
	a = sin(difflon)*cos(lat2);
	b = cos(lat1)*sin(lat2)+(-1)*sin(lat1)*cos(lat2)*cos(difflon);
	
	bear = RAD_TO_DEG*(atan2 ( a, b ));
    return bear;
}

void distance_view(){
	distance_viewer.resize(nofcoors);
	for(int i=0; i<nofcoors; i++){				
		distance_viewer[i] = Distance(coordinates[i][0], coordinates[i][1], current_lat, current_lon);
		cout<<"Distance from coordinate number : "<<i+1<<" is :" <<distance_viewer[i]<<endl;
	}
	for(int i=0; i<nofcoors; i++){
		if (distance_viewer[i]<5){
			i++;
			cout<<"SeDriCa is at the gps coordinate number : "<<i<<endl;
			i--;  
		}
	}
}
	
void strtofcoors(string strcoors){  //function to separate the coordinates and store them in usable form 
	int j=0;
	int len = strcoors.length();
	
	for (int i=0 ;i<len;i++){
		if (strcoors[i] == '(' ) j++;
	}
	
	nofcoors = j;
	
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
	
	process_completion = 1;
}

void data_svr_con(const std_msgs::String::ConstPtr& msg){
	data = msg->data.c_str(); 
	strtofcoors(data);
}

void pos_assign(const vectornav::vn200_msg::ConstPtr& input){
	current_lat = input->LLA.latitude;	
	current_lon = input->LLA.longitude;
}

int main(int argc, char **argv){	
	ros::init(argc, argv, "gpsBearing");
	
	ros::NodeHandle n;

	ros::Subscriber sub  = n.subscribe("/gps_pnts", 10, data_svr_con);
	ros::Subscriber sub2 = n.subscribe("/vn200_monitor", 10, pos_assign);

	ros::Publisher  pub  = n.advertise<std_msgs::Float32>("/desired_bearing", 1);
	ros::Publisher pub2  = n.advertise<globalpath::bearing_distance>("/global_plan",1); 
	ros::Publisher pub3  = n.advertise<std_msgs::Float32>("/goal_distance",1); 
	
	ros::Rate loop_rate(20);
	
	globalpath::bearing_distance bearing_distance;
	std_msgs::Float32 bearing;
	std_msgs::Float32 goaldistance;
	
	while (ros::ok()){	
		if (process_completion == 1){		 			 						
			bearingDistance.resize(nofcoors-1);
			
			bearing_distance.header.stamp=ros::Time::now();
			bearing_distance.header.frame_id = "bd";
			bearing_distance.bearing.clear();
			bearing_distance.distance.clear();
			
			for(int i=0; i<(nofcoors-1); i++){
				bearingDistance[i].resize(2);
				bearingDistance[i][0] = Bearing(coordinates[i][0], coordinates[i][1], coordinates[i+1][0], coordinates[i+1][1]); 
				bearingDistance[i][1] = Distance(coordinates[i][0], coordinates[i][1], coordinates[i+1][0], coordinates[i+1][1]);
				bearing_distance.bearing.push_back(bearingDistance[i][0]);
				bearing_distance.distance.push_back(bearingDistance[i][1]); 
				
				cout<<"Move on a compass bearing of : "<<bearingDistance[i][0]<<" degrees"<<" for distance : "<<bearingDistance[i][1]<<" metres"<<endl;
			}

			pub2.publish(bearing_distance);

			distance_view();
		
			for(int i=0; i<nofcoors; i++){
				if (distance_viewer[i]<7){
					if (i == (nofcoors-1)){
						bearing.data = bearingDistance[0][0];
					}
					else bearing.data = bearingDistance[i+1][0];;
					
					if (i == (nofcoors-1)){
						goaldistance.data = distance_viewer[0];
					}
					else goaldistance.data = distance_viewer[i+1];
				}
			}

			cout<<endl;
		
			pub.publish(bearing);
			pub3.publish(goaldistance);
		}
		process_completion = 0;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

