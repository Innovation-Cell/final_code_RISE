#include "image_properties.h"
#include "videofeed/lane.h"
#include "videofeed/multi_lane.h"
#include "videofeed/calib.h"
#include "videofeed/multi_calib.h"
#include "math.h"

using namespace cv;
using namespace std;

string WINDOW = "Occupancy-Gird";

videofeed::multi_lane Multi_Lane_Data;
videofeed::lane Lane_Data;

Mat src;

ros::Subscriber pub_Lanedata;

void constructgrid(const videofeed::multi_calib& message)
{
	src = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);

	//cout<<"No of lanes: "<< message.num_of_lanes<<endl;
	Multi_Lane_Data.num_of_lanes = 0;

	for (int i = 0; i < message.num_of_lanes; ++i)
	{
		for (int j = 0; j < message.Lanes[i].number; ++j)
		{
			float ground_x, ground_y;
			int pix_x = message.Lanes[i].x[j];
			int pix_y = message.Lanes[i].y[j];
			int occ_x, occ_y;

			if ((pix_y > 3*image_height/4) && (pix_y < image_height))
			{
				ground_x = (float)((float)pix_x - (float)image_width/2)*(Calib_Dist_x[3]/Calib_Pix_x[3]);
				ground_y = Calib_Begin_Dist_y[3] + (float)((float)image_height - (float)pix_y)*(Calib_Dist_y[3]/Calib_Pix_y[3]);
			}

			else if ((pix_y > 2*image_height/4) && (pix_y < 3*image_height/4))
			{
				ground_x = (float)((float)pix_x - (float)image_width/2)*(Calib_Dist_x[2]/Calib_Pix_x[2]);
				ground_y = Calib_Begin_Dist_y[2] + (float)((float)(3*image_height/4) - (float)pix_y)*(Calib_Dist_y[2]/Calib_Pix_y[2]);
			}

			else if ((pix_y > image_height/4) && (pix_y < 2*image_height/4))
			{
				ground_x = (float)((float)pix_x - (float)image_width/2)*(Calib_Dist_x[1]/Calib_Pix_x[1]);
				ground_y = Calib_Begin_Dist_y[1] + (float)((float)(2*image_height/4) - (float)pix_y)*(Calib_Dist_y[1]/Calib_Pix_y[1]);
			}

			else
			{
				ground_x = (float)((float)pix_x - (float)image_width/2)*(Calib_Dist_x[0]/Calib_Pix_x[0]);
				ground_y = Calib_Begin_Dist_y[0] + (float)((float)(image_height/4) - (float)pix_y)*(Calib_Dist_y[0]/Calib_Pix_y[0]);
			}

			Lane_Data.dist.push_back(sqrt(ground_x*ground_x + ground_y*ground_y));

			if ((ground_x != 0) && (ground_y != 0))
			{
				if (atan(ground_y/ground_x) < 0)
					Lane_Data.theta.push_back(-CV_PI/2 - atan(ground_y/ground_x));
				else
					Lane_Data.theta.push_back(CV_PI/2 - atan(ground_y/ground_x));
			
				int step = map_width/occ_grid_width;
				occ_x = (-1)*map_width/2;
				
				while(occ_x + step < ground_x)
				{
					occ_x = occ_x + step;
				};

				occ_x = (occ_x + map_width/2)/step;

				occ_y = map_length;
				while(occ_y - step > ground_y)
				{
					occ_y = occ_y - step;
				};

				occ_y = (map_length - occ_y)/step;

				occ_x = occ_x + occ_y;
				occ_y = occ_x - occ_y;
				occ_x = occ_x - occ_y;

				src.at<uchar>(occ_x, occ_y) = 255;

				Lane_Data.number++;
			}
		}

		Multi_Lane_Data.Lanes.push_back(Lane_Data);
		Multi_Lane_Data.num_of_lanes++;
	}

	waitKey(1);
	imshow(WINDOW, src);

	Lane_Data.dist.clear();
	Lane_Data.theta.clear();
	Lane_Data.number = 0;

	Multi_Lane_Data.Lanes.clear();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Lane_Occupancy_Grid");

	ros::NodeHandle nh;

	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

	pub_Lanedata = nh.subscribe("/caliberation", 1, constructgrid);

	ros::Rate loop_rate(5);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO("videofeed::occupancygrid.cpp::No error.");
}