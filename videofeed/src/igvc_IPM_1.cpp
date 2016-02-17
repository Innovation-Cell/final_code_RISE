/*
 * TODO : 
 * A variable to exit the processing
 * 
 */
//Includes all the headers necessary to use the most common public pieces of the ROS system.
//Use image_transport for publishing and subscribing to images in ROS
//Use cv_bridge to convert between ROS and OpenCV Image formats
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
//Include headers for OpenCV Image processing
//Include headers for OpenCV GUI handling
#include "image_properties.h"
#include "videofeed/lane.h"
#include "videofeed/multi_lane.h"
#include "videofeed/calib.h"
#include "videofeed/multi_calib.h"
#include "math.h"
 
using namespace std;
using namespace cv;
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

int leftB = 0;
int topB = 0;
int top_left[no_of_sections];
int top_right[no_of_sections];
int track_vleft[no_of_sections];
int track_vright[no_of_sections];
int track_horiz[no_of_sections];

Mat stitch;

string inputNode = "/camera/image_raw";
string outputNode = "/camera/image_processed";

//This function is called everytime a new image is published
void ipm(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("videofeed::igvc_IPM.cpp::cv_bridge exception: %s", e.what());
        return;
    }

	cv::Mat src = cv_ptr->image;
	imshow("Source",src);

	stitch = Mat::zeros(Size(image_width + image_height/no_of_sections, image_height), CV_8UC3);

	for (int i = 0; i < no_of_sections; ++i)
	{
		Mat cropped;
		cropped = src(cv::Rect(leftB, topB + i*src.rows/no_of_sections, src.cols - leftB, src.rows/no_of_sections - topB)).clone();

		Mat padded(cropped.rows, cropped.cols + cropped.rows, cropped.type());
		padded.setTo(Scalar::all(0));

		Mat temp;
		temp = cropped.clone();
		resize(cropped, cropped, padded.size());
		cropped.setTo(Scalar::all(0));
		temp.copyTo(padded(Rect(Point(temp.rows/2, 0), temp.size())));
		cropped = padded.clone();


		Point2f c1[4];
		Point2f c2[4];
	
		c1[0] = Point(0, 0);
		c1[1] = Point(temp.cols, 0);
		c1[2] = Point(0, temp.rows);
		c1[3] = Point(temp.cols , temp.rows);

		c2[0] = Point(cropped.rows/2 + top_left[i], 0);
		c2[1] = Point(cropped.rows/2 + src.cols - top_right[i], 0);
		c2[2] = Point(cropped.rows/2 + Calib_Bottom_Left[i], cropped.rows);
		c2[3] = Point(cropped.rows/2 + src.cols - Calib_Bottom_Right[i], cropped.rows);
		
		Mat transform(3, 3, CV_32FC1);
		transform = getPerspectiveTransform(c1, c2);
	
		Mat color_dst = temp.clone();
		Size size = cropped.size();
		warpPerspective(temp, color_dst, transform, size);
		cv::waitKey(10);

		char name[50];
		sprintf(name, "IPM%d", i+1);

		Mat roi = stitch(Rect(leftB, topB + i*stitch.rows/no_of_sections, stitch.cols - leftB, stitch.rows/no_of_sections - topB));
		color_dst.copyTo(roi);

		line(color_dst, Point(track_vleft[i], 0), Point(track_vleft[i], color_dst.cols - 1), Scalar(255, 0, 255));
		line(color_dst, Point(track_vright[i], 0), Point(track_vright[i], color_dst.cols - 1), Scalar(255, 0, 255));
		line(color_dst, Point(0, track_horiz[i]), Point(color_dst.cols - 1, track_horiz[i]), Scalar(255, 0, 255));
		
		imshow(name, color_dst);		
	}

	stitch = stitch(Rect(leftB + stitch.rows/(2*no_of_sections), topB, image_width, image_height));
	imshow("stitch", stitch);
	cv_ptr->image = stitch;

	pub.publish(cv_ptr->toImageMsg());

	stitch.release();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Inverse_Perspective_Mapping");
	ros::NodeHandle nh;
	namedWindow("Source", CV_WINDOW_AUTOSIZE);

	for (int i = 0; i < no_of_sections; ++i)
	{
		char name[10];
		sprintf(name, "IPM%d", i+1);
		namedWindow(name, CV_WINDOW_AUTOSIZE);
	}

	
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera/image_processed", 1, ipm);

	for (int i = 0; i < no_of_sections; ++i)
	{
		char Ipm[50];
		sprintf(Ipm, "IPM%d", i+1);

		top_left[i] = 0;
		top_right[i] = 0;
		track_vleft[i] = 154;
		track_vright[i] = 589;
		track_horiz[i] = 50;
		createTrackbar("Calib_Bottom_Left", Ipm, &Calib_Bottom_Left[i], 2000);
		createTrackbar("Calib_Bottom_Right", Ipm, &Calib_Bottom_Right[i], 2000);
		createTrackbar("top_left", Ipm, &top_left[i], 2000);
		createTrackbar("top_right", Ipm, &top_right[i], 2000);
		createTrackbar("track_vleft", Ipm, &track_vleft[i], 2000);
		createTrackbar("track_vright", Ipm, &track_vright[i], 2000);
		createTrackbar("track_horiz", Ipm, &track_horiz[i], 2000);
	}

	//createTrackbar("Lambda", "IPM", &lambda, 1000);
	//createTrackbar("Lambda", "IPM", &lambda, 1000);
	//	imshow("Source",color_dst);createTrackbar("Lambda", "IPM", &lambda, 1000);
	
	pub = it.advertise("/camera/Corrected", 1);
	ros::spin();

	destroyAllWindows(); 
}