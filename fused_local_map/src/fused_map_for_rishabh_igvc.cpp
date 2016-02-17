#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <stdio.h>
#include <iostream>

#include "fused_local_map/customOccupancy.h"
#include "fused_local_map/custocc.h"
#include "fused_local_map/Char.h"
#include "fused_local_map/pix.h"
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

ros::Publisher pub;

void fusionCallback(const fused_local_map::custocc::ConstPtr& msg)
{
	
	
	int height = 400;
	int width = 800;

	nav_msgs::OccupancyGrid fusedGrid1;
	fusedGrid1.header.stamp = ros::Time::now();
	fusedGrid1.header.frame_id = "/fused";
	fusedGrid1.info.resolution = 0.05;
	fusedGrid1.info.origin.position.x = 0.0;
	fusedGrid1.info.origin.position.y = 0.0;
	fusedGrid1.info.origin.position.z = 0.0;
	fusedGrid1.info.origin.orientation.x = 0.0;
	fusedGrid1.info.origin.orientation.y = 0.0;
	fusedGrid1.info.origin.orientation.z = 0.0;
	fusedGrid1.info.origin.orientation.w = 1.0;
	fusedGrid1.info.width = 800;
	fusedGrid1.info.height = 400;
	fusedGrid1.info.map_load_time = ros::Time::now();

	
	std_msgs::Int8MultiArray map;
	
	for (int i = 0; i < height; ++i)
	{
	    for (int j = 0; j < width; ++j)
		{
			map.data.push_back(0);
		}
	}
    
    for (int i = 0; i < height; ++i)
	{
	    for (int j = 0; j < width; ++j)
		{
			//cout<<"hi"<<endl;
			int a = msg->data[i*width+j].d;			
			if(a == 1) map.data[i*width+j] = 1;
		}
	}

	int radius_in_cm = 40;
    int radius_in_cells = radius_in_cm/5;
    int xRadius = radius_in_cells;
    int yRadius = radius_in_cells;
    int radius = radius_in_cells;

     int blown_map[height][width];
     for(int i=0; i<height; i++)
	 {
	 	for(int j=0; j<width; j++)
	 	{
	 		blown_map[i][j] = 0;
	 	}
	 }

    for(int i=0; i<height; i++)
	 {
	 	for(int j=0; j<width; j++)
	 	{
	 		int a = map.data[i*width+j];
             if(a == 1 && msg->data[i*width+j].id.data == 'W')
             {
             	blown_map[i][j] = 1;
             	for(int k=0; k<xRadius; k++)
	              {
				 	for(int l=0; l<yRadius; l++)
				 	{
				 		if(k*k + l*l < radius*radius){
                           if((i-k)>=0 && (j-l)>=0 && (i-k)<height && (j-l)<width) blown_map[i-k][j-l] = 1;
                           if((i-k)>=0 && (j+l)>=0 && (i-k)<height && (j+l)<width) blown_map[i-k][j+l] = 1;
                           if((i+k)>=0 && (j-l)>=0 && (i+k)<height && (j-l)<width) blown_map[i+k][j-l] = 1;
                           if((i+k)>=0 && (j+l)>=0 && (i+k)<height && (j+l)<width) blown_map[i+k][j+l] = 1;
                        }

				 	}
				}
             }

             if(a == 1 && msg->data[i*width+j].id.data == 'L')
             {
             	blown_map[i][j] = 2;
             	for(int k=0; k<xRadius; k++)
	              {
				 	for(int l=0; l<yRadius; l++)
				 	{
				 		if(k*k + l*l < radius*radius){
                           if((i-k)>=0 && (j-l)>=0 && (i-k)<height && (j-l)<width) blown_map[i-k][j-l] = 2;
                           if((i-k)>=0 && (j+l)>=0 && (i-k)<height && (j+l)<width) blown_map[i-k][j+l] = 2;
                           if((i+k)>=0 && (j-l)>=0 && (i+k)<height && (j-l)<width) blown_map[i+k][j-l] = 2;
                           if((i+k)>=0 && (j+l)>=0 && (i+k)<height && (j+l)<width) blown_map[i+k][j+l] = 2;
                        }

				 	}
				}
             }
         }
     }

     int blown_map_reversed[height][width];
      for(int i=0; i<height; i++)
	 {
	 	for(int j=0; j<width; j++)
	 	{
	 		blown_map_reversed[i][j] = blown_map[height-1-i][width-1-j];
	 	}
	 }
  
	 std_msgs::Int8MultiArray map_req_finally;
	 for(int i=0; i<height; i++)
	 {
	 	for(int j=0; j<width; j++)
	 	{
	 		int a = blown_map_reversed[i][j];
	 		if(a == 0 || a == 1)
	 		map_req_finally.data.push_back(a);
	 	    if(a == 2)
	 	    map_req_finally.data.push_back(a-1);
          }
      }


	Mat fused_map = Mat::zeros(height, width, CV_8UC3);
	for (int i = 0; i < height; ++i)
	{
	    for (int j = 0; j < width; ++j)
		{
			int a = blown_map_reversed[i][j];
			if(a==0 || a==1)
			{
				fused_map.at<Vec3b>(i, j)[0] = a * 255;
			 	fused_map.at<Vec3b>(i, j)[1] = a * 255;
			 	fused_map.at<Vec3b>(i, j)[2] = a * 255;
			}
			if(a == 2)
			{
				fused_map.at<Vec3b>(i, j)[0] = (a-1) * 0;
			 	fused_map.at<Vec3b>(i, j)[1] = (a-1) * 0;
			 	fused_map.at<Vec3b>(i, j)[2] = (a-1) * 255;
			}
		}
	}

    waitKey(1);
    imshow("fusedGrid", fused_map);

    fusedGrid1.data = map_req_finally.data;
    pub.publish(fusedGrid1);
    map.data.clear();
    map_req_finally.data.clear();
    fusedGrid1.data.clear();
   // map_col.data.clear();

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fused_occ_ris");
	ros::NodeHandle nh;
	ros::NodeHandle n;

	ros::Rate loop_rate(25); 

	namedWindow("fusedGrid", CV_WINDOW_NORMAL);

	ros::Subscriber fused_sub= nh.subscribe("/scan/OccGrdfull", 1, fusionCallback);
	pub = n.advertise<nav_msgs::OccupancyGrid>("/scan/fusedOccGrd", 1);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();    
	}

	return 0;
}