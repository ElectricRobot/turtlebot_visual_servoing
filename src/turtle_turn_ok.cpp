#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <vector>
#include <ctime>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
/*#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
*/

#include <visp/vpMeLine.h>


//using namespace std;
//using namespace cv;

static const std::string TOPIC_NAME = "camera/rgb/image_raw";
static const std::string DEPTH_TOPIC_NAME = "camera/depth/image_raw";
cv::Mat mat, imgsat, mapDraw;
int h_min, h_max, lowThreshold;
int const max_lowThreshold = 100;
double min,max;
cv::RNG rng(12345);
ros::Publisher pub;
int it_cnt; //iteration number

time_t rawtime;

void on_trackbar_min( int, void* )
{

}

void on_trackbar_max( int, void* )
{
}

void on_trackbar_canny( int, void* )
{
}

std::string showTime()
{
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
  std::string str(buffer);

  return str;
}

cv::Mat rotateImg (cv::Mat src, int angle)
{
	cv::Point2f center(src.cols/2.0F, src.rows/2.0F);
	cv::Mat rot = getRotationMatrix2D(center, angle, 1.0);
	cv::Mat dst;
    cv::Rect bbox = cv::RotatedRect(center,src.size(), angle).boundingRect();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - center.x;
    rot.at<double>(1,2) += bbox.height/2.0 - center.y;
    cv::warpAffine(src, dst, rot, bbox.size());
	return dst;
}

void cannyDetector(cv::Mat src, cv::Mat &imgMap)
{
	int ratio = 3;
	int kernel_size = 3;
	cv::Mat srcGray, srcHsv, cannyOutput;
	std::vector<std::vector<cv::Point> > contours;

	// from color to gray
	cv::cvtColor(src, srcGray, cv::COLOR_BGR2GRAY);
	// from color to hsv
	cv::cvtColor(src, srcHsv, cv::COLOR_BGR2HSV); 

	/// Reduce noise with a kernel 3x3
	cv::blur( srcGray, srcGray, cv::Size(3,3) );

	/// Canny detector
	cv::Canny( srcGray, cannyOutput, lowThreshold, lowThreshold*ratio, kernel_size );

	/// Find contours
	cv::findContours(cannyOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// Select orange contours
	imgMap = cv::Mat::zeros( srcGray.size(), srcGray.type() );
	cv::Rect bRect = cv::Rect(0,0,0,0);
	for( int i = 0; i< contours.size(); i++ )
	{
		if (cv::contourArea(contours[i]) > 0.00001*src.rows*src.cols)
		{
			bRect = cv::boundingRect(contours[i]);
			cv::inRange(srcHsv(bRect), cv::Scalar(h_min,50,50), cv::Scalar(h_max,255,255), imgMap(bRect));
		}
	}
	cv::erode(imgMap, imgMap, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1)));
	cv::dilate(imgMap, imgMap, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

	//Draw contours
	cv::Mat drawing = cv::Mat::zeros( cannyOutput.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, i, cv::Scalar(0,0,255), 1, 8);
	}
	
}

void lineParam(cv::Rect roi, std::vector<cv::Vec4i> lines, int &p1x, int &p1y, int &p2x, int &p2y)
{
	int p1x_acc, p1y_acc, p2x_acc, p2y_acc;
	p1x_acc=0; p1y_acc=0; p2x_acc=0; p2y_acc=0;
	int k = 0;

	// calculate average line
	for( size_t i = 0; i < lines.size(); i++ )
	{
		cv::Vec4i l = lines[i];
		if (l[0] < l[2] && l[0] < 30) //select vertical lines which start in image top
		{
			p1x_acc = p1x_acc + l[0] + roi.x; p1y_acc = p1y_acc+ l[1] + roi.y; p2x_acc = p2x_acc + l[2] + roi.x; p2y_acc = p2y_acc+ l[3]+roi.y;
			cv::line(mapDraw, cv::Point (l[0] + roi.x,l[1] + roi.y), cv::Point(l[2] + roi.x,l[3] + roi.y), cv::Scalar(0, 255, 0), 2, 8); //draw line
			k++;
		}
	}
	if (k != 0){p1x = p1x_acc/k; p1y = p1y_acc/k; p2x = p2x_acc/k; p2y = p2y_acc/k;}
}

void houghTransform(cv::Mat img, float* &alpha)
{

	// rotate for hough detection
	img = rotateImg(img, 90);
	// canny edge detection
	cv::Mat imgMap;
	cannyDetector(img, imgMap); // retrives a binary image with contours of orange spots
	cvtColor(imgMap, mapDraw, CV_GRAY2BGR); //image for drawing

	// detect line ROI1
	cv::Rect roi1 = cv::Rect(0, 160, 120, 320);
	cv::rectangle(mapDraw, cv::Point(roi1.x,roi1.y), cv::Point(roi1.x+roi1.width,roi1.y+roi1.height), cv::Scalar( 0, 55, 255 ), +1, 4 );
	std::vector<cv::Vec4i> lines;
	HoughLinesP(imgMap(roi1), lines, 1, CV_PI/180, 50, 50, 20 );
	int p1x, p1y, p2x, p2y; // line start and final point

	if (lines.size() > 0) 
	{
		lineParam(roi1, lines, p1x, p1y, p2x, p2y);
		*alpha =  atan((float)(p2y-p1y)/(p2x-p1x));
		cv::line(mapDraw, cv::Point (p1x,p1y), cv::Point(p2x,p2y), cv::Scalar(0, 255, 0), 2, 8); //draw line
	}
	else *alpha = 999;
	*alpha =  atan((float)(p2y-p1y)/(p2x-p1x));

	//result visualization
	mapDraw = rotateImg(mapDraw, -90);
	cv::imshow("hough", mapDraw);
	
}

void twistPublisher(float* angle)
{
	std::string str;
	if (*angle != 999)
	{
		// angle control (if kp=1): -90 => -1 and 90=> 1
		int kp = 2;
		geometry_msgs::Twist twist;
		twist.angular.z = kp*(*angle/(0.5*3.1416));
		str = showTime();
		std::cout<< it_cnt<<" "<<clock()<<" " << *angle<<" " << twist.angular.z <<std::endl;
		pub.publish(twist);
	}
}


void imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
	try {
	it_cnt++; // acc iteration number
	//convert ROS image to CV image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

	// HOUGH transform
	float* angle;
	houghTransform(cv_ptr->image, angle);
	twistPublisher(angle);
	
	//visualization source image
	cv::imshow("source", cv_ptr->image);
	cv::waitKey(10);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {

	h_min = 0;
	h_max = 7;
	lowThreshold = 0;
	std::cout<< "N Time Angle(rd) Wz(rd/s)" <<std::endl;
	it_cnt = 0;
	std::clock_t start = clock();

    ros::init(argc, argv, "image_transport_subscriber");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_NAME, 1, imageCallBack);
	pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	ros::Rate loop_rate(10);


    cv::namedWindow("source", CV_WINDOW_NORMAL);
    cv::namedWindow("hough", CV_WINDOW_NORMAL);
	cv::createTrackbar( "min Hue", "hough", &h_min, 180, on_trackbar_min );
	cv::createTrackbar( "max Hue", "hough", &h_max, 180, on_trackbar_max );
	cv::createTrackbar( "canny:", "hough", &lowThreshold, max_lowThreshold, on_trackbar_canny );

    ros::spin();
    cv::startWindowThread();

    cv::destroyWindow("source");
    cv::destroyWindow("hough");
    cv::destroyWindow("canny");
    ros::shutdown();
    return 0;
}
