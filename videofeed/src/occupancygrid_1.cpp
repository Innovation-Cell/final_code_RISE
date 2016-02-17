#include "image_properties.h"
#include "videofeed/lane.h"
#include "videofeed/multi_lane.h"
#include "videofeed/calib.h"
#include "videofeed/multi_calib.h"
#include "math.h"

using namespace cv;
using namespace std;

string WINDOW = "Occupancy-Gird";

videofeed::multi_calib outdata;
videofeed::calib Lane_Data;

ros::Subscriber sub_Lanedata;
ros::Publisher pub_Lanedata;

Mat src;
Mat src_rgb;

Point Get_closest(Point pt, std::vector<Point> vec, int &k)
{
	float diffx = pt.x - vec[0].x;
	float diffy = pt.y - vec[0].y;
	k = 0;
	float min_dist = sqrt( diffx*diffx + diffy*diffy);

	for (int i = 0; i < vec.size(); ++i)
	{
		float diffx = pt.x - vec[i].x;
		float diffy = pt.y - vec[i].y;

		float dist = sqrt( diffx*diffx + diffy*diffy);

		if (dist < min_dist)
		{
			min_dist = dist;
			k = i;
		}
	}

	return vec[k];
}

std::vector<Point> Lane_arrange(std::vector<Point> vec)
{
	std::vector<Point> Lane;

	int sum_x = 0;
	int sum_y = 0;
	
	Lane = vec;

	for (int i = 0; i < Lane.size() - 1; ++i)
	{
		sum_x = sum_x + abs(Lane[i].x - Lane[i+1].x);
		sum_y = sum_y + abs(Lane[i].y - Lane[i+1].y);
	}

	if (sum_x >= sum_y)
	{
		if (Lane[0].x > Lane[Lane.size() - 1].x)
		{
			reverse(Lane.begin(), Lane.end());
		}
	}
	else
	{
		if (Lane[0].y < Lane[Lane.size() - 1].y)
		{
			reverse(Lane.begin(), Lane.end());
		}
	}

	return Lane;
}


void Arrange(std::vector<Point> critical_poly_points, std::vector<std::vector<Point> > &Interpolated_Lane_points)
{
	for (int i = 0; i < Interpolated_Lane_points.size(); ++i)
	{
		if (Interpolated_Lane_points[i].size() != 0)
		{
			int k = 0;
			Point start = critical_poly_points[i];
			circle(src_rgb, critical_poly_points[i], 4, Scalar(0, 255, 255));
		
			std::vector<Point> newlane;
			newlane.push_back(start);
			Interpolated_Lane_points[i].erase(Interpolated_Lane_points[i].begin() + k);

			while(Interpolated_Lane_points[i].size() != 0)
			{
				Point next = Get_closest(newlane[newlane.size() - 1], Interpolated_Lane_points[i], k);
				newlane.push_back(next);
				Interpolated_Lane_points[i].erase(Interpolated_Lane_points[i].begin() + k);
			};

			Interpolated_Lane_points[i] = Lane_arrange(newlane);
		}
	}
}

void arrange_spline(std::vector<std::vector<Point> > &Lanes)
{
	std::vector<Point> critical_poly_points;

	for (int i = 0; i < Lanes.size(); ++i)
	{
		int max = Lanes[i][0].y;
		int k = 0;

		for (int j = 0; j < Lanes[i].size(); ++j)
		{
			if (max < Lanes[i][j].y)
			{
				max = Lanes[i][j].y;
				k = j;
			}
		}

		critical_poly_points.push_back(Lanes[i][k]);
		circle(src_rgb, Lanes[i][k], 4, Scalar(0, 0, 255));
	}

	Arrange(critical_poly_points, Lanes);
}


void constructgrid(const videofeed::multi_calib& message)
{
	std::vector<std::vector<Point> > Lane_points;
	src = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
	src_rgb = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC3);

	Lane_points.resize(message.num_of_lanes);

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

			if ((ground_x != 0) && (ground_y >= 0))
			{			
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

				src.at<uchar>(occ_y, occ_x) = 255;

				Lane_points[i].push_back(Point(occ_x, occ_y));
			}
		}
	}

	for (int i = 0; i < Lane_points.size(); ++i)
	{
		if (Lane_points[i].size() == 0)
		{
			Lane_points.erase(Lane_points.begin() + i);
			i--;
		}
	}

	arrange_spline(Lane_points);

	for (int i = 0; i < Lane_points.size(); ++i)
	{
		Lane_Data.x.clear();
		Lane_Data.y.clear();
		Lane_Data.number = 0;
		
		for (int j = 0; j < Lane_points[i].size(); ++j)
		{
			Lane_Data.x.push_back(Lane_points[i][j].x);
			Lane_Data.y.push_back(Lane_points[i][j].y);
			Lane_Data.number++;				
		}	
		
		outdata.Lanes.push_back(Lane_Data);
		outdata.num_of_lanes++;
	}


	for (int i = 0; i < Lane_points.size(); ++i)
	{
		circle(src_rgb, Lane_points[i][0], 4, Scalar(255, 255, 0));
		circle(src_rgb, Lane_points[i][Lane_points[i].size() - 1], 4, Scalar(255, 255, 255));

		for (int j = 0; j < Lane_points[i].size() - 1; ++j)
		{
			line(src_rgb, Lane_points[i][j], Lane_points[i][j+1], Scalar(255, 255, 255), 1);
		}
	}


	waitKey(1);
	imshow(WINDOW, src);
	imshow("src_rgb", src_rgb);

	pub_Lanedata.publish(outdata);

	outdata.Lanes.clear();
	outdata.num_of_lanes = 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Lane_Occupancy_Grid");

	ros::NodeHandle nh;

	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	namedWindow("src_rgb", CV_WINDOW_AUTOSIZE);

	sub_Lanedata = nh.subscribe("/caliberation", 1, constructgrid);
	pub_Lanedata = nh.advertise<videofeed::multi_calib>("/Interpolater", 100);

	ros::Rate loop_rate(5);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO("videofeed::occupancygrid.cpp::No error.");
}