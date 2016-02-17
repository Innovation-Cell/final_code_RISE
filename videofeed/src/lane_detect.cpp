#include "image_properties.h"
#include "videofeed/lane.h"
#include "videofeed/multi_lane.h"
#include "videofeed/calib.h"
#include "videofeed/multi_calib.h"
#include "math.h"

using namespace cv;
using namespace std;

videofeed::multi_calib outdata;
videofeed::calib Lane_Data;

ros::Publisher pub_outdata;

Mat src, dst;
Mat hough_output;

int RLow = 200;
int GLow = 200;
int BLow = 200;
int RHigh = 255;
int GHigh = 255;
int BHigh = 255;
int epsilon = 6;

int const max_RLow = 255;
int const max_GLow = 255;
int const max_BLow = 255;
int const max_RHigh = 255;
int const max_GHigh = 255;
int const max_BHigh = 255;
int const max_epsilon = 20;

int const stage_iter_count = 10;

int area_threshold = 100.0;
int const max_area_threshold = 1000.0;

int min_threshold = 50;
int p_threshold = 10;
int p_length = 10;
int p_maxgap = 10;

int const max_trackbar = 150;


/**
*  @Find which mean is nearest to the data point
**/
int find_cluster(Vec4i segment, std::vector<Vec4i> means)
{
	//cout<<"find_cluster summoned \n";
	int cluster = 0;

	float diffx = (segment[0] + segment[2])/2 - (means[0][0] + means[0][2])/2;
	float diffy = (segment[1] + segment[3])/2 - (means[0][1] + means[0][3])/2;

	float min_dist = sqrt( diffx*diffx + diffy*diffy );

	//cout<<"Data considered ("<< segment[0] <<", "<< segment[1] <<") - ("<< segment[2] <<", "<< segment[3] <<") \n";

	for (size_t i = 0; i < means.size(); ++i)
	{
		float diffx = (segment[0] + segment[2])/2 - (means[i][0] + means[i][2])/2;
		float diffy = (segment[1] + segment[3])/2 - (means[i][1] + means[i][3])/2;

		//Distance based on the distance between mid points of line segments.
		float dist = sqrt( diffx*diffx + diffy*diffy );

		if (dist < min_dist)
		{
			min_dist = dist;
			cluster = i;
		}
	}

	//cout<<"cluster = "<< cluster <<endl;
	//cout<<"find_cluster ended \n";
	return cluster;
}

/**
*  @Find the nearest cluster to a given cluster
**/
int nearest(Vec4i mean, std::vector<Vec4i> means)
{
	//cout<<"nearest summoned \n";
	int near_cluster = 0;

	float diffx = (mean[0] + mean[2])/2 - (means[0][0] + means[0][2])/2;
	float diffy = (mean[1] + mean[3])/2 - (means[0][1] + means[0][3])/2;

	float min_dist = sqrt( diffx*diffx + diffy*diffy );

	//cout<<"Data considered ("<< mean[0] <<", "<< mean[1] <<") - ("<< mean[2] <<", "<< mean[3] <<") \n";

	for (size_t i = 0; i < means.size(); ++i)
	{
		float diffx = (mean[0] + mean[2])/2 - (means[i][0] + means[i][2])/2;
		float diffy = (mean[1] + mean[3])/2 - (means[i][1] + means[i][3])/2;

		//Distance based on the distance between mid points of line segments.
		float dist = sqrt( diffx*diffx + diffy*diffy );

		if ((dist < min_dist) && (mean != means[i]))
		{
			min_dist = dist;
			near_cluster = i;
		}
	}

	//cout<<"near_cluster = "<< near_cluster <<endl;
	//cout<<"nearest ended \n";
	return near_cluster;
}

/**
*  @Function to check whether cluster-means found are appropriate
**/
bool check(std::vector<Vec4i> lines, std::vector<Vec4i> cluster_means, float &prev_sil_coeff)
{
	//cout<<"check summoned \n";

	if (lines.size() == 0)
	{
		//cout<<"check ended \n";
		return true;
	}

	std::vector<std::vector<Vec4i> > cluster;
	float overall_sil_coeff = 0;
	float tot_dist = 0;

	cluster.resize(cluster_means.size());

	//Arrange data into clusters based on nearest cluster means.
	for (size_t i = 0; i < lines.size(); ++i)
	{
		int j = find_cluster(lines[i], cluster_means);
		cluster[j].push_back(lines[i]);
	}

	for (size_t i = 0; i < cluster.size(); ++i)
	{
		for (size_t j = 0; j < cluster[i].size(); ++j)
		{
			float A = 0;
			float B = 0;
			float sil_coeff = 0;
			int num_A = 0;				
			int num_B = 0;

			//Average distance of a point from all other points in the same cluster.
			for (size_t k = 0; k < cluster[i].size(); ++k)
			{
				float diffx = (cluster[i][k][0] + cluster[i][k][2])/2 - (cluster[i][j][0] + cluster[i][j][2])/2;
				float diffy = (cluster[i][k][1] + cluster[i][k][3])/2 - (cluster[i][j][1] + cluster[i][j][3])/2;

				float dist = sqrt( diffx*diffx + diffy*diffy );

				A = A + dist;
				num_A++;
			}

			if (num_A != 0)
				A = A/num_A;		

			//cout<<"A = "<< A <<endl;

			//Average distance of a point from all other points in the nearest cluster.
			int near_cluster = nearest(cluster_means[i], cluster_means);

			for (size_t k = 0; k < cluster[near_cluster].size(); ++k)
			{
				float diffx = (cluster[near_cluster][k][0] + cluster[near_cluster][k][2])/2 - (cluster[i][j][0] + cluster[i][j][2])/2;
				float diffy = (cluster[near_cluster][k][1] + cluster[near_cluster][k][3])/2 - (cluster[i][j][1] + cluster[i][j][3])/2;

				float dist = sqrt( diffx*diffx + diffy*diffy );

				B = B + dist;
				num_B++;
			}

			if (num_B != 0)
				B = B/num_B;

			//cout<<"B = "<< B <<endl;

			if ((A > B) && (A != 0) && (B != 0))
				sil_coeff = abs(B - A)/A;
			else if((A != 0) && (B != 0))
				sil_coeff = abs(B - A)/B;


			float diffx = cluster[i][j][0] - cluster[i][j][2];
			float diffy = cluster[i][j][1] - cluster[i][j][3];

			//Average weighted by line segment lengths.
			float dist = sqrt( diffx*diffx + diffy*diffy );

			overall_sil_coeff = overall_sil_coeff + sil_coeff*dist;

			//cout<<"sil_coeff = "<< sil_coeff <<endl;

			tot_dist = tot_dist + dist;
		}
	}

	//Overall silhouette coefficient evaluvated.
	if (tot_dist != 0)
		overall_sil_coeff = overall_sil_coeff/tot_dist;
	

	//cout<<"overall_sil_coeff = "<< overall_sil_coeff <<" prev_sil_coeff = "<< prev_sil_coeff <<endl;

	/*
		If silhouette coefficient is larger than that found earlier,
		better approximation possible.
		This calculated value is stored for further comparisons.

		Else this is the maximised silhouette coefficient.
	*/

	if (overall_sil_coeff >= prev_sil_coeff)	
	{
		prev_sil_coeff = overall_sil_coeff;
		//cout << overall_sil_coeff <<endl;
		//cout<<"check ended \n";
		return false;
	}
	else
	{
		prev_sil_coeff = overall_sil_coeff;
		//cout << overall_sil_coeff <<" *"<<endl;
		//cout<<"check ended \n";
		return true;
	}
}


bool contains_j(std::vector<int> random_result, int j)
{
	bool flag = false;

	for (size_t i = 0; i < random_result.size(); ++i)
	{
		if (j == random_result[i])
		{
			flag = true;
			break;
		}
	}

	return flag;
}

/**
* @Find the cluster means
**/
void find_means(std::vector<Vec4i> lines, std::vector<Vec4i> &cluster_means, int k)
{
	//cout<<"find_means summoned \n";
	int count = 0;
	int iter_count = 0;
	std::vector<int> random_result;

	//Choose random line segments as initial cluster means.
	for (size_t i = 0; i < cluster_means.size(); ++i)
	{
		if (lines.size() != 0)
		{
			int j = rand() % lines.size();

			if (contains_j(random_result, j))
			{
				i--;
				continue;
			}
			
			//int j = rand() % lines.size();
			
			cluster_means[i][0] = lines[j][0];
			cluster_means[i][1] = lines[j][1];
			cluster_means[i][2] = lines[j][2];
			cluster_means[i][3] = lines[j][3];

			random_result.push_back(j);
		}

		else
		{
			cluster_means[i][0] = 0;
			cluster_means[i][1] = 0;
			cluster_means[i][2] = 0;
			cluster_means[i][3] = 0;
		}

		//cout<<"Initial cluster_mean ("<< cluster_means[i][0] <<", "<< cluster_means[i][1] <<") - ("<< cluster_means[i][2] <<", "<< cluster_means[i][3] <<") \n";
	}

	//Execute the loop till count is not equal to k.
	while((count != k) && (iter_count < 2*stage_iter_count))					
	{
		std::vector<Vec4i> new_cluster_means;
		std::vector<std::vector<Vec4i> > cluster;
		
		//Declare vectors to find new cluster means.
		cluster.resize(k);
		new_cluster_means.resize(k);

		count = 0;

		//Arrange data into clusters based on nearest cluster means.
		for (size_t i = 0; i < lines.size(); ++i)
		{
			int j = find_cluster(lines[i], cluster_means);		
			cluster[j].push_back(lines[i]);
		}

		for (size_t i = 0; i < cluster.size(); ++i)
		{
			//Find new cluster means.
			new_cluster_means[i][0] = 0;				
			new_cluster_means[i][1] = 0;
			new_cluster_means[i][2] = 0;
			new_cluster_means[i][3] = 0;
			
			float tot_dist = 0;

			for (size_t j = 0; j < cluster[i].size(); ++j)
			{
				float diffx = cluster[i][j][0] - cluster[i][j][2];
				float diffy = cluster[i][j][1] - cluster[i][j][3];

				float dist = sqrt( diffx*diffx + diffy*diffy );		

				//Average weighted by line segment lengths.
				new_cluster_means[i][0] = new_cluster_means[i][0] + cluster[i][j][0] * dist;
				new_cluster_means[i][1] = new_cluster_means[i][1] + cluster[i][j][1] * dist;
				new_cluster_means[i][2] = new_cluster_means[i][2] + cluster[i][j][2] * dist;
				new_cluster_means[i][3] = new_cluster_means[i][3] + cluster[i][j][3] * dist;

				tot_dist = tot_dist + dist;

				//cout<<"Cluster_mean ("<< new_cluster_means[i][0] <<", "<< new_cluster_means[i][1] <<") - ("<< new_cluster_means[i][2] <<", "<< new_cluster_means[i][3] <<") \n";
				//cout<<"tot_dist = "<< tot_dist <<endl;
			}
			
			//Average found.
			if (tot_dist != 0)
			{
				new_cluster_means[i][0] = new_cluster_means[i][0]/tot_dist;
				new_cluster_means[i][1] = new_cluster_means[i][1]/tot_dist;
				new_cluster_means[i][2] = new_cluster_means[i][2]/tot_dist;
				new_cluster_means[i][3] = new_cluster_means[i][3]/tot_dist;
			}
		}

		//Compare new cluster means and old cluster means.
		for (size_t i = 0; i < cluster_means.size(); ++i)
		{
			//cout<<"("<< cluster_means[i][0] <<", "<< cluster_means[i][1] <<") - ("<< cluster_means[i][2] <<", "<< cluster_means[i][3] <<")\n";
			//cout<<"("<< new_cluster_means[i][0] <<", "<< new_cluster_means[i][1] <<") - ("<< new_cluster_means[i][2] <<", "<< new_cluster_means[i][3] <<")\n";

			int error[4];
			error[0] = cluster_means[i][0] - new_cluster_means[i][0];
			error[1] = cluster_means[i][1] - new_cluster_means[i][1];
			error[2] = cluster_means[i][2] - new_cluster_means[i][2];
			error[3] = cluster_means[i][3] - new_cluster_means[i][3];
			
			if((abs((float)error[0]) < 1) && (abs((float)error[1]) < 1) && (abs((float)error[2]) < 1) && (abs((float)error[3]) < 1) && (iter_count < stage_iter_count))
				count++;
			
			else if((abs((float)error[0]) < 2) && (abs((float)error[1]) < 2) && (abs((float)error[2]) < 2) && (abs((float)error[3]) < 2) && (iter_count > stage_iter_count))
				count++;
			
			else			
				break;
		}

		//Initialise new cluster means as old ones.
		for (size_t i = 0; i < new_cluster_means.size(); ++i)
		{
			cluster_means[i][0] = new_cluster_means[i][0];
			cluster_means[i][1] = new_cluster_means[i][1];
			cluster_means[i][2] = new_cluster_means[i][2];
			cluster_means[i][3] = new_cluster_means[i][3];

			//cout<<"("<< cluster_means[i][0] <<", "<< cluster_means[i][1] <<") - ("<< cluster_means[i][2] <<", "<< cluster_means[i][3] <<")\n";
		}

		iter_count++;
		//cout<<iter_count<<endl;
		//cout<<"count = "<< count <<" k = "<< k <<endl;
	}

	for (size_t i = 0; i != cluster_means.size(); ++i);
	//cout<<"("<< cluster_means[i][0] <<", "<< cluster_means[i][1] <<") - ("<< cluster_means[i][2] <<", "<< cluster_means[i][3] <<")\n";
	//cout<<"find_means ended \n";
}


void Associate_Lane(std::vector<std::vector<Point> > poly, std::vector<std::vector<Point> >& Interpolated_Lane_points)
{
	//cout<<"Associate_Lane() called \n";
	//cout<<"poly.size() = "<<poly.size()<<"\t"<<"Interpolated_Lane_points.size() = "<<Interpolated_Lane_points.size()<<endl;

	for (int i = 0; i < poly.size(); ++i)
	{
		if (i >= Interpolated_Lane_points.size())
		{
			poly.erase(poly.begin() + i);
			i--;			
		}

		for (int j = 0; j < Interpolated_Lane_points.size(); ++j)
		{
			//cout<<"Interpolated_Lane_points["<<j<<"].size() = "<<Interpolated_Lane_points[j].size()<<endl;
			if (pointPolygonTest(poly[i], Interpolated_Lane_points[j][0], false) >= 0)
			{
				std::vector<Point> temp = Interpolated_Lane_points[i];
				Interpolated_Lane_points[i] = Interpolated_Lane_points[j];
				Interpolated_Lane_points[j] = temp;
			}
		}
	}
	//cout<<"Associate_Lane() ends \n";
}

Point Get_closest(Point pt, std::vector<Point> vec, int &k)
{
	//cout<<"Get_closest() called \n";
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
	//cout<<"Get_closest() ends \n";
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
		reverse(Lane.begin(), Lane.end());
	}
	else
	{
		if (Lane[0].y < Lane[Lane.size() - 1].y)
		reverse(Lane.begin(), Lane.end());
	}

	return Lane;
}

void Arrange(std::vector<Point> critical_poly_points, std::vector<std::vector<Point> >& Interpolated_Lane_points)
{
	//cout<<"Arrange() called \n";
	//cout<<"Interpolated_Lane_points.size() = "<<Interpolated_Lane_points.size()<<"\t"<<"critical_poly_points.size() = "<<critical_poly_points.size()<<endl;

	for (int i = 0; i < Interpolated_Lane_points.size(); ++i)
	{
		//cout<<"Interpolated_Lane_points["<<i<<"].size() = "<<Interpolated_Lane_points[i].size()<<endl;

		if (Interpolated_Lane_points[i].size() != 0)
		{
			int k = 0;
			Point start = Get_closest(critical_poly_points[i], Interpolated_Lane_points[i], k);
		
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
	//cout<<"Arrange() ended \n";
}

bool Lane_in_contour(std::vector<Point> contour, std::vector<std::vector<Point> > Lane_points)
{
	bool flag = false;

	for (int i = 0; i < Lane_points.size(); ++i)
	{
		int count = 0;
		
		for (int j = 0; j < Lane_points[i].size(); ++j)
		{
			if(pointPolygonTest(contour, Lane_points[i][j], false) >= 0)
				count++;
		}

		if (count >= Lane_points[i].size()/2)
		{
			flag = true;
			break;
		}
	}

	return flag;
}

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

void arrange_spline(std::vector<std::vector<Point> > Lane_points, std::vector<std::vector<Point> >& Interpolated_Lane_points)
{
	//cout<<"arrange_spline() called \n";
	std::vector<std::vector<Point> > contours;
	std::vector<std::vector<Point> > poly;
	std::vector<Vec4i> hierarchy;

	for (int i = 0; i < Lane_points.size(); ++i)
	{
		if (Lane_points[i].size() == 0)
		{
			Lane_points[i].clear();
			Lane_points.erase(Lane_points.begin() + i);
			i--;
		}
	}

	findContours(dst, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	Interpolated_Lane_points = Lane_points;
	//cout<<"Interpolated_Lane_points.size() = "<<Interpolated_Lane_points.size()<<"\t"<<"Lane_points.size() = "<<Lane_points.size()<<endl;

	for (int i = 0; i < contours.size(); ++i)
	{
		if (Lane_in_contour(contours[i], Lane_points))
		{
			std::vector<Point> vec;
			approxPolyDP(Mat(contours[i]), vec, (double)epsilon, true);

			Point* pt = &vec[0];

			fillConvexPoly(dst, pt, vec.size(), Scalar(100, 0, 100));

			for (int j = 0; j < vec.size(); ++j)
			//circle(dst, vec[j], 2, Scalar(255, 255, 255));

			poly.push_back(vec);
		}
	}

	Associate_Lane(poly, Interpolated_Lane_points);

	std::vector<Point> critical_poly_points;
	critical_poly_points.clear();

	for (int i = 0; i < poly.size(); ++i)
	{
		double min_angle = poly_angle(poly[i], 0);
		Point pt = poly[i][0];

		for (int j = 0; j < poly[i].size(); ++j)
		{
			double angle = poly_angle(poly[i], j);

			if (angle < min_angle)
			{
				min_angle = angle;
				pt = poly[i][j];
			}
		}

		critical_poly_points.push_back(pt);

		//circle(dst, pt, 4, Scalar(255, 0, 0));
	}

	Arrange(critical_poly_points, Interpolated_Lane_points);

	for (int i = 0; i < Interpolated_Lane_points.size(); ++i)
	circle(hough_output, Interpolated_Lane_points[i][0], 4, Scalar(255, 255, 255));
	//cout<<"arrange_spline() ends \n"<<endl;
}

std::vector<Point> convert(std::vector<Vec4i> means)
{
	std::vector<Point> Lane_points;

	if ((means[0][0] != 0) && (means[0][1]))
	Lane_points.push_back(Point(means[0][0], means[0][1]));

	for (size_t i = 0; i < means.size(); ++i)
	{
		Point pt = Point(0.5*(means[i][0]+means[i][2]), 0.5*(means[i][1]+means[i][3]));
		
		if ((pt.x != 0) && (pt.y != 0))
		Lane_points.push_back(pt);
	}

	if ((means[means.size()-1][2] != 0) && (means[means.size()-1][3] != 0))
	Lane_points.push_back(Point(means[means.size()-1][2], means[means.size()-1][3]));

	return Lane_points;
}

/**
*  @K-Means Algorithm Applied
**/
std::vector<Point> kmeans(std::vector<Vec4i> lines)
{
	//cout<<"kmeans summoned \n";
	int k = 1;
	float prev_sil_coeff = 0;

	std::vector<Vec4i> prev_means;
	std::vector<Point> Lane_points;

	//Continue loop till infinity.
	while(true)
	{
		std::vector<Vec4i> cluster_means;
		cluster_means.resize(k);
		//cout<<"k = "<< k <<endl;

		//Find cluster means of k clusters.
		find_means(lines, cluster_means, k);
		
		//If cluster means are appropriate, exit the loop.
		if (check(lines, cluster_means, prev_sil_coeff) || (k >= lines.size()) )
		{
			for (size_t i = 0; i != prev_means.size(); ++i)
			{
				Vec4i l = prev_means[i];
				//cout<<"("<< l[0] <<", "<< l[1] <<") - ("<< l[2] <<", "<< l[3] <<") \n";
				//line( hough_output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 255), 3, CV_AA);
			}
			break;
		}

		prev_means = cluster_means;
		cluster_means.clear();
		k++;
	}

	if (prev_means.size() != 0)
	Lane_points = convert(prev_means);

	prev_means.clear();

	return Lane_points;
	//cout<<"kmeans ended \n";
}



bool contains(RotatedRect rect, Point pt)
{
	Point2f vertices[4];
	rect.points(vertices);

	bool chk = true;

	for (int i = 0; i < 4; ++i)
	{
		float m = (vertices[i].y - vertices[(i+1) % 4].y)/(vertices[i].x - vertices[(i+1) % 4].x);
		float c = vertices[i].y - m*vertices[i].x;

		float val_pt = pt.y - m*pt.x - c;
		float val_center = rect.center.y - m*rect.center.x - c;

		if (val_pt * val_center > 0)
			chk = true;
		else
		{
			chk = false;
			break;
		}
	}

	return chk;
}


void threshold(int, void*)
{
	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	std::vector<Vec4i> p_lines;
	
	inRange(src, Scalar(RLow, GLow, BLow), Scalar(RHigh, GHigh, BHigh), dst);
	//imwrite("/home/ajaykumar/Desktop/Mahindra/Lane_Interpolation/image1.jpg", dst);
	Mat detected_contours = Mat::zeros(dst.size(), CV_8UC1);

	//Find contours of the image
	findContours(dst, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	if (contours.size() != 0)
	{
		for (int idx = 0; idx >= 0; idx = hierarchy[idx][0])
		{
			if (contourArea(contours[idx]) >= area_threshold)
			{
				Scalar color(100, 100, 100);
				drawContours(detected_contours, contours, idx, color, CV_FILLED, 8, hierarchy);
			}
		}
	}

	//imwrite("/home/ajaykumar/Desktop/Mahindra/Lane_Interpolation/image2.jpg", detected_contours);

	//detected_contours = dst.clone();
	HoughLinesP(detected_contours, p_lines, 1, CV_PI/180, min_threshold + p_threshold, p_length, p_maxgap);

	for (size_t i = 0; i < p_lines.size(); ++i)
	{
		if (p_lines[i][1] >= p_lines[i][3])
		{
			int temp_col = p_lines[i][0];
			p_lines[i][0] = p_lines[i][2];
			p_lines[i][2] = temp_col;

			int temp_row = p_lines[i][1];
			p_lines[i][1] = p_lines[i][3];
			p_lines[i][3] = temp_row;
		}
	}

	std::vector<std::vector<Vec4i> > Grouped_lines;
	std::vector<RotatedRect> rect;
	std::vector<std::vector<Point> > Lane_points;
	std::vector<std::vector<Point> > Interpolated_Lane_points;

	//Find rectangle approximations for the contours
	if (contours.size() != 0)
	{
		for (int idx = 0; idx >= 0; idx = hierarchy[idx][0])
		rect.push_back(minAreaRect(contours[idx]));
	}
	
	Grouped_lines.resize(rect.size());

	cvtColor(detected_contours, detected_contours, CV_GRAY2BGR);

	for (size_t j = 0; j < p_lines.size(); ++j)
	{
		bool flag = false;

		for (size_t i = 0; i < rect.size(); ++i)
		{   
			//Find midpoint of each line.
			Point pt = Point(0.5*(p_lines[j][0] + p_lines[j][2]), 0.5*(p_lines[j][1] + p_lines[j][3]));

			if (contains(rect[i], pt) && !flag)
			{   
				//If midpoint lies inside the rectangle, line to be part of that rectangle's group of lines
				Grouped_lines[i].push_back(p_lines[j]);
				flag = true;
			}

			Point2f vertices[4];
			rect[i].points(vertices);

			for (int k = 0; k < 4; ++k)
			line(detected_contours, vertices[k], vertices[(k+1) % 4], Scalar(255, 255, 255));
		}
	}


	cvtColor(detected_contours, detected_contours, CV_BGR2GRAY);

	hough_output = Mat::zeros(src.size(), CV_8UC3);

	for (int i = 0; i < p_lines.size(); ++i)
	{
		Vec4i l = p_lines[i];
		line( hough_output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
	}

	Lane_points.resize(Grouped_lines.size());

	for (int i = 0; i < Grouped_lines.size(); ++i)
	Lane_points.push_back(kmeans(Grouped_lines[i]));

	arrange_spline(Lane_points, Interpolated_Lane_points);

	outdata.num_of_lanes = 0;
	outdata.Lanes.clear();

	for (int i = 0; i < Interpolated_Lane_points.size(); ++i)
	{
		Lane_Data.number = 0;
		for (int j = 0; j < Interpolated_Lane_points[i].size() - 1; ++j)
		{
			circle(hough_output, Interpolated_Lane_points[i][j], 2, Scalar(255, 255, 255));

			Lane_Data.x.push_back(Interpolated_Lane_points[i][j].x);
			Lane_Data.y.push_back(Interpolated_Lane_points[i][j].y);
			Lane_Data.number++;

			line(hough_output, Interpolated_Lane_points[i][j], Interpolated_Lane_points[i][j+1], Scalar(255, 255, 255), 3);
		}

		outdata.Lanes.push_back(Lane_Data);

		Lane_Data.x.clear();
		Lane_Data.y.clear();
		Lane_Data.number = 0;
		outdata.num_of_lanes++;
	}

	imshow("detected_contours", detected_contours);
	imshow("HoughLines_Demo", hough_output);
	imshow("Source_Image", src);
	imshow("DST", dst);

	pub_outdata.publish(outdata);

	detected_contours.release();
}

namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "Image Processed";
image_transport::Publisher pub;
 
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	createTrackbar("RLow", "detected_contours", &RLow, max_RLow, threshold);
	createTrackbar("RHigh", "detected_contours", &RHigh, max_RHigh, threshold);
	createTrackbar("GLow", "detected_contours", &GLow, max_GLow, threshold);
	createTrackbar("GHigh", "detected_contours", &GHigh, max_GHigh, threshold);
	createTrackbar("BLow", "detected_contours", &BLow, max_BLow, threshold);
	createTrackbar("BHigh", "detected_contours", &BHigh, max_BHigh, threshold);
	createTrackbar("area_threshold", "detected_contours", &area_threshold, max_area_threshold, threshold);

	createTrackbar( "Threshold", "HoughLines_Demo", &p_threshold, max_trackbar, threshold);
	createTrackbar( "Maxgap", "HoughLines_Demo", &p_maxgap, max_trackbar, threshold);
	createTrackbar( "Length", "HoughLines_Demo", &p_length, max_trackbar, threshold);

	createTrackbar( "Epsilon", "DST", &epsilon, max_epsilon, threshold);

	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("videofeed::lane_detect.cpp::cv_bridge exception: %s", e.what());
		return;
    }     
 
	cv::imshow(WINDOW, cv_ptr->image);
	cv::waitKey(3);

	src = cv_ptr->image;

	namedWindow("Source_Image", CV_WINDOW_AUTOSIZE);
	
	namedWindow("HoughLines_Demo", CV_WINDOW_AUTOSIZE);

	threshold(0, 0);

	pub.publish(cv_ptr->toImageMsg());
}
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Lane_Detection");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

	namedWindow("Source_Image", CV_WINDOW_AUTOSIZE);
	namedWindow("detected_contours", CV_WINDOW_AUTOSIZE);
	namedWindow("HoughLines_Demo", CV_WINDOW_AUTOSIZE);
	namedWindow("DST", CV_WINDOW_AUTOSIZE);

	image_transport::Subscriber sub = it.subscribe("camera/Corrected", 1, imageCallback);

	pub_outdata = nh.advertise<videofeed::multi_calib>("/Interpolater", 100);

	pub = it.advertise("camera/final_output", 1);
	ros::Rate loop_rate(5);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("videofeed::lane_detect.cpp::No error.");
}