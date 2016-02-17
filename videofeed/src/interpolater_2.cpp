#include "image_properties.h"
#include "videofeed/lane.h"
#include "videofeed/multi_lane.h"
#include "videofeed/calib.h"
#include "videofeed/multi_calib.h"
#include "math.h"

using namespace cv;
using namespace std;

int degree = 3;

ros::Subscriber sub_interpolate;
ros::Publisher pub_pix;

videofeed::calib Lane_data;
videofeed::multi_calib Interpolated_Lane_data;

std::vector<std::vector<Point> > Lane_points;

Mat src;

double poly_angle(std::vector<Point> poly, int idx)
{
	//cout<<"poly_angle() called \n";
	double angle;
	double m1, m2;
	if (idx == 0)
	{
		//Points [size-1], [0], [1]
		m1 = atan2((poly[0].y - poly[poly.size() - 1].y),(poly[0].x- poly[poly.size() - 1].x));
		m2 = atan2((poly[1].y - poly[0].y),(poly[1].x - poly[0].x));
	}
	else if (idx == poly.size()-1)
	{
		//Points [size-2], [size-1], [0]
		m1 = atan2((poly[poly.size() - 1].y - poly[poly.size() - 2].y),(poly[poly.size() - 1].x- poly[poly.size() - 2].x));
		m2 = atan2((poly[0].y - poly[poly.size() - 1].y),(poly[0].x - poly[poly.size() - 1].x));
	}
	else
	{
		//Points [idx-1], [idx], [idx+1]
		m1 = atan2((poly[idx].y - poly[idx - 1].y),(poly[idx].x- poly[idx - 1].x));
		m2 = atan2((poly[idx + 1].y - poly[idx].y),(poly[idx + 1].x - poly[idx].x));
	}

	angle = abs(m1) - abs(m2);
	//cout<<"poly_angle() ends \n";
	return angle;
}


Point deBoor(int k, int degree, int i, float x, std::vector<float> knots, std::vector<Point> ctrlPoints)
{
	
	if(i < 0)
	return ctrlPoints[0];
	
	if(k == 0)
	{
		//cout<<"k = "<< k <<" i = "<< i <<endl;
		//cout<<"ctrlPoints["<< i <<"] = ("<< ctrlPoints[i].x <<" , "<< ctrlPoints[i].y <<") \n";
		return ctrlPoints[i];
	}
	else
	{
		float alpha;
		if(abs(knots[i+degree+1-k] - knots[i]) < 0.1)
		alpha = 0;
		else
		alpha = (x - knots[i])/(knots[i+degree+1-k] - knots[i]);
		
		Point pt = deBoor(k-1, degree, i, x, knots, ctrlPoints)*alpha + deBoor(k-1, degree, i-1, x, knots, ctrlPoints)*(1 - alpha);
		//cout<<"pt = ("<< pt.x <<" , "<< pt.y <<")  alpha = "<< alpha <<"  "; 
		//cout<<"k = "<< k <<" i = "<< i <<endl;
		return pt; 
	}
}

int insert(float x, std::vector<float> knot)
{
	int index;
	
	for(int i=0; i<knot.size(); i++)
	if((knot[i] <= x) && (x < knot[i+1]))
	{
		index = i;
		break;
	}
	
	//cout<<"Index = "<< index <<endl;
	
	return index;
}

std::vector<Point> Get_Spline(std::vector<Point> control_points)
{
	std::vector<Point> spline_points;
	std::vector<float> knot;

	knot.resize(control_points.size() + degree);

	for (int i = 0; i < knot.size(); ++i)
	knot[i] = i*10;

	for (int i = 0; i < degree; ++i)
	knot[knot.size() - i - 1] = knot.size()*10;
	
	//for(float x=knot[0]; x<knot[knot.size()-1]; x=x+5)
	for(float x=knot[0]; x<knot[knot.size()-1]; x=x+(knot[knot.size()-1]-knot[0])/40)
	{
		//cout<<"x = "<< x <<"  ";
		Point pt = deBoor(degree, degree, insert(x, knot), x, knot, control_points);
		spline_points.push_back(pt);
	}

	return spline_points;
}


void Interpolate(std::vector<std::vector<Point> > &Interpolated_Lane_points)
{
	std::vector<std::vector<Point> > Lane_points;
	Lane_points = Interpolated_Lane_points;
	Interpolated_Lane_points.clear();

	if (Lane_points.size() != 0)
	{
		std::vector<Point> starting;
		starting = Lane_points[0];
		int k = 0;

		for (int i = 1; i < Lane_points.size(); ++i)
		{
			if (Lane_points[i][0].y > starting[0].y)
			{
				starting = Lane_points[i];
				k = i;
			}
		}

		std::vector<Point> temp;

		temp = Lane_points[0];
		Lane_points[0] = Lane_points[k];
		Lane_points[k] = temp; 	
	}
	
	for (int i = 0; i < Lane_points.size(); ++i)
	{
		std::vector<Point> spline;
		spline = Get_Spline(Lane_points[i]);

		double avg_angle = 0;
		double dev_angle = 0;
		double no_of_angle = 0;

		for (int j = spline.size() - 6; j < spline.size() - 1; ++j)
		{
			avg_angle += poly_angle(spline, j);
			no_of_angle++;
		}

		avg_angle = avg_angle/no_of_angle;

		for (int j = spline.size() - 6; j < spline.size() - 1; ++j)
		{
			double term = poly_angle(spline, j);
			dev_angle += (term - avg_angle)*(term - avg_angle);
		}

		dev_angle = dev_angle/(no_of_angle - 1);

		for (int j = 0; j < Lane_points.size(); ++j)
		{
			if (i != j)
			{
				Lane_points[i].push_back(Lane_points[j][0]);

				double angle = poly_angle(Lane_points[i], Lane_points[i].size() - 2);

				if ((angle > avg_angle - 2*dev_angle) && (angle < avg_angle + 2*dev_angle))
				{
					Lane_points[i].erase(Lane_points[i].begin() + Lane_points[i].size() - 1);

					Lane_points[i].insert(Lane_points[i].end(), Lane_points[j].begin(), Lane_points[j].end());

					Lane_points.erase(Lane_points.begin() + j);

					i--;

					break;
				}
			}
		}
	}

	Interpolated_Lane_points.resize(Lane_points.size());

	for (int i = 0; i < Lane_points.size(); ++i)
	{
		Interpolated_Lane_points[i] = Get_Spline(Lane_points[i]);

		for (int j = 0; j < Interpolated_Lane_points[i].size(); ++j)
		{
			src.at<uchar>(Interpolated_Lane_points[i][j].y, Interpolated_Lane_points[i][j].x) = 255;
		}
	}
}


void interpolate(const videofeed::multi_calib& message)
{
	Interpolated_Lane_data.num_of_lanes = 0;
	
	src = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);

	for (int i = 0; i < message.num_of_lanes; ++i)
	{
		std::vector<Point> temp;

		for (int j = 0; j < message.Lanes[i].number; ++j)
		{
			temp.push_back(Point(message.Lanes[i].x[j], message.Lanes[i].y[j]));
			//circle(src, temp[temp.size() - 1], 2, Scalar(255, 255, 255));
		}

		//circle(src, temp[0], 2, Scalar(255, 255, 255));

		assert(temp.size() > 0);

		Lane_points.push_back(temp);
		temp.clear();	
	}

	Interpolate(Lane_points);

	for (int i = 0; i < Lane_points.size(); ++i)
	{
		Lane_data.number = 0;
	
		circle(src, Lane_points[i][0], 2, Scalar(255, 255, 255));

		for (int j = 0; j < Lane_points[i].size(); ++j)
		{
			Lane_data.x.push_back(Lane_points[i][j].x);
			Lane_data.y.push_back(Lane_points[i][j].y);
			Lane_data.number++;
		}

		Interpolated_Lane_data.Lanes.push_back(Lane_data);

		Lane_data.x.clear();
		Lane_data.y.clear();

		Interpolated_Lane_data.num_of_lanes++;
	}

	//pub_pix.publish(Interpolated_Lane_data);

	for (int i = 0; i < Lane_points.size(); ++i)
		Lane_points[i].clear();

	Lane_points.clear();

	waitKey(1);

	imshow("src", src);

	Interpolated_Lane_data.num_of_lanes = 0;
	Interpolated_Lane_data.Lanes.clear();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Interpolation");

	ros::NodeHandle nh;

	namedWindow("src", CV_WINDOW_AUTOSIZE);

	sub_interpolate = nh.subscribe("/Interpolater", 1, interpolate);

	ros::Rate loop_rate(5);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO("videofeed::interpolater.cpp::No error.");
}