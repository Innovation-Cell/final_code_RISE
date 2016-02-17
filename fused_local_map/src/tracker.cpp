#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <tf/transform_listener.h>

#include <stdio.h>
#include <vector>
#include <iostream>
#include <string> 
#include <sstream>
#include <algorithm>
#include <stack>
#include <math.h>

#include "tracking/customOccupancy.h"
#include "tracking/custocc.h"
#include "tracking/Char.h"
#include "tracking/pix.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Int8.h"


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

const float res = 0.25;        //resolution of LiDAR 
const int num_values = 1080;
int counter = 0;
int clusterID = 0;
sensor_msgs::LaserScan curr;
ros::Publisher pub;
ros::Publisher pub1;
ros::Subscriber lane_sub;
tracking::custocc lane;
int flag = 0;

	


pair<float, float> convertToCart(int i, float dist)
{
	float ang = i * 0.25 * 3.14 / 180;
	float x = cos(ang) * dist;
	float y = sin(ang) * dist;

	return make_pair(x, y);
}


void addNoise(std_msgs::Float64MultiArray& map, int height, int width)
{
	int arrayForError[height+2][width+2];
	for(int i=1; i<= height; i++)
	{
		for(int j=1; j<= width; j++)
		{
			arrayForError[i][j] = map.data[(i-1)*height+(j-1)];
		}
	}
	
	for(int i=0; i<= height+1; i++)
	{
		arrayForError[0][i] = 0;
		arrayForError[height+1][i] = 0;
		arrayForError[i][0] = 0;
		arrayForError[i][height+1] = 0;
	}
   float gaussianKernel[3][3] = {{0.0256,0.1088,0.0256},{0.1088,0.4624,0.1088},{0.0256,0.1088,0.0256}};
   float arrayWithError[height][width];
   for(int i=1; i<= height; i++)
	{
		for(int j=1; j<= width; j++)
		{
			float sum = gaussianKernel[0][0]*arrayForError[i-1][j-1] + gaussianKernel[0][1]*arrayForError[i-1][j] + gaussianKernel[0][2]*arrayForError[i-1][j+1]+
			          gaussianKernel[1][0]*arrayForError[i][j-1] + gaussianKernel[1][1]*arrayForError[i][j] + gaussianKernel[1][2]*arrayForError[i][j+1]+
			          gaussianKernel[2][0]*arrayForError[i+1][j-1] + gaussianKernel[2][1]*arrayForError[i+1][j] + gaussianKernel[2][2]*arrayForError[i+1][j+1];
			arrayWithError[i][j] = sum;
			
		}
		
	}
	for(int i=1; i<= height; i++)
	{
		for(int j=1; j<= width; j++)
		{
			map.data[(i-1)*height+(j-1)] = arrayWithError[i][j];
	     }
		
	}
}

void laneCallback(const nav_msgs::OccupancyGrid& msg)
{
   cout<<"came in"<<endl;
    lane.header.stamp = ros::Time::now();
	//lane.header.frame_id = msg->;
	lane.info.origin.position.x = msg.info.origin.position.x;
	lane.info.origin.position.y = msg.info.origin.position.y;
	lane.info.origin.position.z = msg.info.origin.position.z;
	lane.info.origin.orientation.x = msg.info.origin.orientation.x;
	lane.info.origin.orientation.y = msg.info.origin.orientation.y;
	lane.info.origin.orientation.z = msg.info.origin.orientation.z;
	lane.info.origin.orientation.w = msg.info.origin.orientation.w;
	lane.info.width = msg.info.width;
	lane.info.height = msg.info.height;
	lane.info.resolution = msg.info.resolution;
	lane.info.map_load_time = ros::Time::now();
	for (int i = 0; i < msg.info.height; ++i)
	{
		for (int j = 0; j < msg.info.width; ++j)
		{
			tracking::pix h;
			if(msg.data[i*msg.info.width+j]!=0) h.d = 1;				
			else h.d = 0;
			//printf("%d , ", h.d);

			// printf("%d",msg.data[i*msg.info.width+j]);
			//int b=int(a*(-1));
			// cout<<a<<",";
			//h.d = (int)a;
			//cout<<h.d;
			//if(abs(h.d)==1) cout<<"yeah"<<endl;
            h.id.data = 'L';
            // printf("%d", h.d);;
			lane.data.push_back(h);
			int a = lane.data[i*msg.info.width+j].d;
			// std_msgs::Int8 b;
			//  b.data = lane.data[i*msg.info.width+j].d;
			 //printf("%d", a);
			 //printf("%d , ", b.data);
			// std_msgs::Int8 b;
			// b.data = lane.data[i*msg->info.width+j].d;
			// int c = b.data;
			// cout<<c;
		}
	}
			

	// for (int i = 0; i < msg->info.height; ++i)
	// {
	// 	for (int j = 0; j < msg->info.width; ++j)
	// 	{
	// 		if(lane.data[i*msg->info.width+j].d == 1) cout<<"yeah"<<endl;
	// 	}
	// }
	// flag =1;cout<<"flag= "<<flag;
}


void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	curr.header.frame_id = msg->header.frame_id;
	curr.range_min = 0.01;
	curr.range_max = 20.0;
	curr.scan_time = msg->scan_time;
	curr.angle_increment = msg->angle_increment;
	curr.angle_min = msg->angle_min;
	curr.angle_max = msg->angle_max;
	curr.time_increment = msg->time_increment;
	curr.ranges.resize(num_values);
	//curr.intensities.resize(num_values);
	curr.header.stamp = msg->header.stamp;
	curr.header.seq = msg->header.seq;
	// cout<<"hey"<<endl;
	for (int i = 0; i < num_values; i++)
	{
		curr.ranges[i] = msg->ranges[i];
		//curr.intensities[i] = msg->intensities[i];
	}
    	
	vector<pair<float, float> > cart(num_values, make_pair(0, 0));
	for(int i=0; i<num_values; i++)
	{
		 cart[i] = convertToCart(i - num_values/2 , curr.ranges[i]);
		// cart[i] = convertToCart(i, curr.ranges[i]);
	}
lane.info.width = 200;
	lane.info.height = 400;
	lane.header.stamp = ros::Time::now();
	lane.header.frame_id = "/lane";
	//lane.info.resolution = reso;
	// The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
	lane.info.origin.position.x = 0.0;
	lane.info.origin.position.y = 0.0;
	lane.info.origin.position.z = 0.0;
	lane.info.origin.orientation.x = 0.0;
	lane.info.origin.orientation.y = 0.0;
	lane.info.origin.orientation.z = 0.0;
	lane.info.origin.orientation.w = 1.0;
	for (int i = 0; i < 400; ++i)
	{
		for (int j = 0; j < 200; ++j)
		{
			tracking::pix h;
			h.d = 0;
            h.id.data = 'L';
			lane.data.push_back(h);
		}
	}
	// cout<<"size = "<<lane.data.size();
	float reso = 0.05;
	int height = 800;
	int width = 800;
    int modified_height = 100;
	int modified_width = 100;		
	int xDist = 800;
	int yDist = 800;

    // nav_msgs::OccupancyGrid laserGrid;
    tracking::customOccupancy laserGrid;
	laserGrid.header.stamp = ros::Time::now();
	laserGrid.header.frame_id = "/lidar";
	laserGrid.info.resolution = reso;
	// The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
	laserGrid.info.origin.position.x = 0.0;
	laserGrid.info.origin.position.y = 0.0;
	laserGrid.info.origin.position.z = 0.0;
	laserGrid.info.origin.orientation.x = 0.0;
	laserGrid.info.origin.orientation.y = 0.0;
	laserGrid.info.origin.orientation.z = 0.0;
	laserGrid.info.origin.orientation.w = 1.0;
	laserGrid.info.width = modified_width;
	laserGrid.info.height = modified_height;
	laserGrid.info.map_load_time = ros::Time::now();

	tracking::custocc laserGrid1;
    //nav_msgs::OccupancyGrid laserGrid1;
	laserGrid1.header.stamp = ros::Time::now();
	laserGrid1.header.frame_id = "/lidar";
	laserGrid1.info.resolution = reso;
	// The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
	laserGrid1.info.origin.position.x = 0.0;
	laserGrid1.info.origin.position.y = 0.0;
	laserGrid1.info.origin.position.z = 0.0;
	laserGrid1.info.origin.orientation.x = 0.0;
	laserGrid1.info.origin.orientation.y = 0.0;
	laserGrid1.info.origin.orientation.z = 0.0;
	laserGrid1.info.origin.orientation.w = 1.0;
	laserGrid1.info.width = width;
	laserGrid1.info.height = height;
	laserGrid1.info.map_load_time = ros::Time::now();


    
    std_msgs::Int8MultiArray map;
	// std_msgs::Float64MultiArray map;
	for(int i= width -1; i>= 0; i--)
	{
		for(int j= height - 1; j>= 0; j--)
		{
			map.data.push_back(0);
		}
	}

	for(int i=0; i<num_values; i++)
	{
		int xCoord = round(cart[i].first / reso);
		int yCoord = round(cart[i].second / reso);

        if(curr.ranges[i]!=0)
	       {
	        if(xCoord >= 0)
	        {
	        	if(yCoord >= -yDist/2 && yCoord <= yDist/2)
	        	{
	        		int x = xDist/2 - xCoord;
	        				map.data[x*width + yCoord + yDist/2] = 1;

	        	}
	        }
	    }
	}


	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			tracking::pix h;
			h.d = map.data[i*width+j];
			h.id.data = 'W';
			laserGrid1.data.push_back(h);
		}
	}
    // cout<<"hey"<<endl;


	for(int i= 0; i<400; ++i)
	{
		for(int j= 0; j<200; ++j)
		{
			int a = lane.data[i*200+j].d;
			//cout<<a<<"yes";
   //  // cout<<"hey"<<endl;

			// //cout<<"flag= "<<flag<<endl;
			
	      if(a != 0)
          {
          	cout<<a;
             laserGrid1.data[i*width+300+j].d = 1;
             laserGrid1.data[i*width+300+j].id.data = 'L';
          }
		}
		// cout<<i<<endl;
	}

    

	std_msgs::Float64MultiArray map_req;
	

	 for(int i=0; i<modified_width; i++)
	 {
	 	for(int j=0; j<modified_height; j++)
	 	{
	 		int a = map.data[((300+i)*width) + (350+j)];
	 		map_req.data.push_back(a);
	 	}
	 }

	 //addNoise(map, height, width);
	 addNoise(map_req, modified_height, modified_width);

	 namedWindow("occupancyGrid", CV_WINDOW_NORMAL);
	 namedWindow("occupancyGridfull", CV_WINDOW_AUTOSIZE);

     Mat mapImg = Mat::zeros(modified_height, modified_width, CV_8UC3);
     Mat mapImgfull = Mat::zeros(height, width, CV_8UC3);


 
		map_req.data[9949] = 0.5;
		     for(int i=0; i<modified_width; i++)
			 {
			 	for(int j=0; j<modified_height; j++)
			 	{
			 		 mapImg.at<Vec3b>(i, j)[0] = map_req.data[i*modified_width + j] * 255;
			 		 mapImg.at<Vec3b>(i, j)[1] = map_req.data[i*modified_width + j] * 255;
			 		 mapImg.at<Vec3b>(i, j)[2] = map_req.data[i*modified_width + j] * 255;

			 		}
			 	}

			 	for(int i=0; i<width; i++)
			 {
			 	for(int j=0; j<height; j++)
			 	{
			 		int a = laserGrid1.data[i*width + j].d;
			 		if(laserGrid1.data[i*width + j].id.data == 'W' && laserGrid1.data[i*width + j].d == 1){
			 	     mapImgfull.at<Vec3b>(i, j)[0] = a * 255;
			 		 mapImgfull.at<Vec3b>(i, j)[1] = a * 255;
			 		 mapImgfull.at<Vec3b>(i, j)[2] = a * 255;
                     }
                     if(laserGrid1.data[i*width + j].id.data == 'L' && laserGrid1.data[i*width + j].d == 1){
			 		 mapImgfull.at<Vec3b>(i, j)[0] = a * 0;
			 		 mapImgfull.at<Vec3b>(i, j)[1] = a * 0;
			 		 mapImgfull.at<Vec3b>(i, j)[2] = a * 255;
                     }
			 		}
			 	}
    //    while (waitKey(30) != 27) { // wait for ESC key press
    //        imshow("occupancyGrid", mapImg);
    //    }
    waitKey(1);
    imshow("occupancyGrid", mapImg);
    imshow("occupancyGridfull", mapImgfull);
    // imshow("clusteredGrid", clusterImg);
 
   
	laserGrid.data = map_req.data;
	//laserGrid1.data = map.data;
	pub.publish(laserGrid);
	pub1.publish(laserGrid1);
	lane.data.clear();

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "OccGrd");
	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::NodeHandle nh2;
	ros::Rate loop_rate(25); 

	lane.info.width = 200;
	lane.info.height = 400;
	lane.header.stamp = ros::Time::now();
	lane.header.frame_id = "/lane";
	//lane.info.resolution = reso;
	// The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
	lane.info.origin.position.x = 0.0;
	lane.info.origin.position.y = 0.0;
	lane.info.origin.position.z = 0.0;
	lane.info.origin.orientation.x = 0.0;
	lane.info.origin.orientation.y = 0.0;
	lane.info.origin.orientation.z = 0.0;
	lane.info.origin.orientation.w = 1.0;
	for (int i = 0; i < 400; ++i)
	{
		for (int j = 0; j < 200; ++j)
		{
			tracking::pix h;
			h.d = 0;
            h.id.data = 'L';
			lane.data.push_back(h);
		}
	}
	// cout<<"size = "<<lane.data.size();
	cout<<"yooo"<<endl;

	 // namedWindow("ClusteredGrid", CV_WINDOW_NORMAL);
	 // createTrackbar("xRadius","ClusteredGrid",&xRadius,15,lidarCallback);
    
	lane_sub= nh2.subscribe("/Lane_Occupancy_Grid", 1, laneCallback);
	ros::Subscriber lidar_sub= nh.subscribe("/scan", 1, lidarCallback);


	// pub = n.advertise<nav_msgs::OccupancyGrid>("/scan/OccGrd", 1);
	pub = n.advertise<tracking::customOccupancy>("/scan/OccGrd", 1);
	pub1 = nh1.advertise<tracking::custocc>("/scan/OccGrdfull", 1);
    
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();    
	}
	return 0;
}
