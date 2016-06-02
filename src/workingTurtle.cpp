// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
// cpp
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Visp
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpMeLine.h>
// Personal library
#include <image_processing/color_detection.hpp>

static const std::string TOPIC_NAME = "camera/rgb/image_raw";
cv::Mat mat, imgsat, mapDraw;
int h_min, h_max, lowThreshold;
int const max_lowThreshold = 100;
double min,max;
cv::RNG rng(12345);
ros::Publisher pub;
int it_cnt; //iterator counter
std::ofstream myfile; // file where results will be writen


void on_trackbar_min( int, void* )
{

}

void on_trackbar_max( int, void* )
{
}

void on_trackbar_canny( int, void* )
{
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

	/*Obsolete version for detection of colored contours... date : 	*/

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
	/*// Show in a window
	cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	cv::imshow( "Contours", drawing );
	cv::namedWindow( "imgMap", CV_WINDOW_AUTOSIZE );
	cv::imshow( "imgMap", imgMap );*/
	
}

int lineParam(cv::Rect roi, std::vector<cv::Vec4i> lines, int &p1x, int &p1y, int &p2x, int &p2y)
{
	int p1x_acc, p1y_acc, p2x_acc, p2y_acc;
	p1x_acc=0; p1y_acc=0; p2x_acc=0; p2y_acc=0;
	p1x=0; p1y=0; p2x=0; p2y=0;
	int k = 0;
	std::cout<<"ROI: "<<roi.x<<" "<<roi.x+roi.width<<" "<<" "<<roi.y<<" "<<roi.y+roi.height<<" "<<std::endl;
	std::cout<<"Printing all hough lines... "<<std::endl;

	// calculate average line
	for( size_t i = 0; i < lines.size(); i++ )
	{
		cv::Vec4i l = lines[i];
		std::cout<<"line"<<i<<": "<<l[0]<<" "<<l[1]<<" "<<l[2]<<" "<<l[3]<<std::endl;
		if (l[0] < l[2] && l[0] < 30) //select vertical lines that start on image top
		{
			p1x_acc = p1x_acc + l[0] + roi.x; p1y_acc = p1y_acc+ l[1] + roi.y; p2x_acc = p2x_acc + l[2] + roi.x; p2y_acc = p2y_acc+ l[3]+roi.y;
			cv::line(mapDraw, cv::Point (l[0] + roi.x,l[1] + roi.y), cv::Point(l[2] + roi.x,l[3] + roi.y), cv::Scalar(0, 255, 0), 2, 8); //draw line
			k++;
		}
	}
	if (k != 0){p1x = p1x_acc/k; p1y = p1y_acc/k; p2x = p2x_acc/k; p2y = p2y_acc/k;} // if k != 0 compute mean points
	return k;
}

void lineROI(int &roi_k, cv::Mat img, cv::Mat img_draw, std::vector<std::vector<int> > &linePoints, std::vector<float> &lineAngles)
{
	cv::Rect roi; //roi where line will be detected
	int dh, dw;	// roi size
	dw = 120; dh = 120;

	std::cout<<"roi_k"<<roi_k<<std::endl;

	//create roi
	if (roi_k == 0)
	{	// the first roi is a fixed window
		dw = 120; dh = 320;
		roi = cv::Rect(0, 0.5*img.rows-0.5*dh, dw, dh); 
		cv::rectangle(img_draw, roi, cv::Scalar( 0, 55, 255 ), +1, 4 ); // draw roi
		std::cout<<"roi0draw"<<std::endl;
	}
	else
	{// control roi overflow for others roi
		if(linePoints[roi_k-1][3] + 0.5*dh > img.rows)
		{
			std::cout<<"if1"<<std::endl;
			roi = cv::Rect(linePoints[roi_k-1][2], linePoints[roi_k-1][3], dw, img.rows - linePoints[roi_k-1][3]);
		}
		else if(linePoints[roi_k-1][3] - 0.5*dh < 0)
		{
			std::cout<<"if2"<<std::endl;
			roi = cv::Rect(linePoints[roi_k-1][2], 0, dw, dh);
		}
		else
		{
			std::cout<<"if3"<<std::endl;
			roi = cv::Rect(linePoints[roi_k-1][2], linePoints[roi_k-1][3] - 0.5*dh, dw, dh);
		}
		std::cout<<"roidraw"<<std::endl;
		cv::rectangle(mapDraw, roi, cv::Scalar( 0, 255, 0 ), +1, 4 );
		std::cout<<"roidraw"<<std::endl;
	}


	// line detection by Hough algorithm
	std::vector<cv::Vec4i> lines; // create vector for line points storage
	std::cout<<"hough"<<std::endl;
	HoughLinesP(img(roi), lines, 1, CV_PI/180, 50, 50, 20 ); 
	std::cout<<"hough"<<std::endl;
	std::cout<<"hough # lines: "<<lines.size()<<std::endl;

	if (lines.size() > 0)
	{
		// calculate best line (average line for while...)
		int p1x, p1y, p2x, p2y; // points of line average
		if (lineParam(roi, lines, p1x, p1y, p2x, p2y) > 0) // if a useful line was found...
		{
			float alpha = 999; //if no line is detected, alpha=999
			std::vector<int> linerow;
			linerow.push_back(p1x); linerow.push_back(p1y); linerow.push_back(p2x); linerow.push_back(p2y);
			std::cout<<"linerow"<<" "<<linerow[0]<<" "<<linerow[1]<<" "<<linerow[2]<<" "<<linerow[3]<<" "<<std::endl;
			linePoints.push_back(linerow); // save average line points in vector linePoints
			alpha =  atan((float)(p2y-p1y)/(p2x-p1x)); // calculate angle with vertical
			lineAngles.push_back(alpha); // save angle in vector lineAngles
			std::cout<<"Angle"<<roi_k<<": "<< alpha*(180/3.1416) <<std::endl; //print angle
			cv::line(mapDraw, cv::Point (p1x,p1y), cv::Point(p2x,p2y), cv::Scalar(0, 0, 255), 2, 8); //draw mean line
			std::cout<<"Printing average line in roi "<<roi_k<<std::endl;
			for(int i = roi_k; i < roi_k+1; i++)
			{
					std::cout<<"Line"<<i<<": "<<linePoints[i][0]<<" "<<linePoints[i][1]<<" "<<linePoints[i][2]<<" "<<linePoints[i][3]<<" "<<std::endl;
			}
		}
		else{roi_k = 4;}// if no useful line was found, stop seeking for line...
	}
	else{roi_k = 4;}// if no line was found by Hough, stop seeking for line...

}
void vote(int a)
{
	/*//choose the best line by vote method..
	std::vector<int> thetaVote(90); // resolution de 2 degrees
	std::vector<int> rhoVote(128); // resolution de 5 pixels
	std::fill(thetaVote.begin(), thetaVote.end(), 0);
	std::fill(rhoVote.begin(), rhoVote.end(), 0);
	int k = 0;
	for( size_t i = 0; i < lines.size(); i++ )
	{
		k = (int) lines[i][1]/2;
		thetaVote[k] = thetaVote[k] + 1;
	}*/
	
	// draw lines and p1 p2 average...
	// attention: p1 below p2 above
}
a
void lineDetector(cv::Mat imgSrc, float &alpha, float &lineEnd)
{

	// rotate image needed for hough detection
	imgSrc = rotateImg(imgSrc, 90);
	// detection of orange contours
	cv::Mat imgMapColor, imgMapMorpho, imgDraw;
	imgDraw  = cv::Mat::zeros( imgSrc.size(), imgSrc.type() );
	int h_min, h_max, m_e, m_d;
	h_min = 0; h_max = 10; // orange min and max hue
	m_e = 5; m_d = 15; // erode and dilate morphological kernels
	ml::findColoredContour(imgSrc, imgMapColor, imgMapMorpho, h_min, h_max, m_e, m_d);

	// parameters of line
	int roi_k; // roi counter (from 0 to 3)
	std::vector<std::vector<int> > linePoints; // point coordinates of the segemented line
	std::vector<float> lineAngles; // anlges of the segemented line
	
	// find line in ROI
	roi_k = 0; 	std::cout<<std::endl<<"Start line detector"<<std::endl;
	while(roi_k < 4)
	{
		lineROI(roi_k, imgMapMorpho, imgDraw, linePoints, lineAngles);
		roi_k++;
	}
	if(linePoints.size() > 0) // print line found
	{
		std::cout<<"Printing lines angles ending points... "<<std::endl;
		for(int i = 0; i < linePoints.size(); i++){
			std::cout<<"Line "<<i<<": "<<linePoints[i][2]<<" "<<linePoints[i][3]<<" "<<std::endl;
			std::cout<<"Angle "<<i<<": "<<lineAngles[i]<<std::endl;
		}
		alpha = lineAngles[0];
		lineEnd = linePoints[linePoints.size() - 1][2];
	}
	else
	{
	std::cout<<"No line found"<<std::endl;
		alpha = 999;
		lineEnd = 999;
	}

	//result visualization
	imgDraw = rotateImg(imgDraw, -90);
	//cv::imshow("source", imgSrc);
	cv::imshow("hough", imgDraw); cv::waitKey(10);
	
}

void lineTraker(cv::Mat I)
{
 	vpImage<unsigned char> img;
    vpImageConvert::convert(I, img);

	std::cout << "VISP..." << std::endl;

	#if defined(VISP_HAVE_X11)
		vpDisplayX d(img, 0, 0, "Camera view");
	#elif defined(VISP_HAVE_GDI)
		vpDisplayGDI d(img, 0, 0, "Camera view");
	#else
		std::cout << "No image viewer is available..." << std::endl;
	#endif

    vpDisplay::setTitle(img, "My image");
    vpDisplay::display(img);
    vpDisplay::flush(img);

	/*vpMe me;
	me.setRange(25);
	me.setThreshold(15000);
	me.setSampleStep(10);
	vpMeLine line;
	line.set Me(&me);
	line.set Display(vpMeSite::RANGE_RESULT);
	line.initTracking(img);
	*/

}


void twistPublisher(float angle, float angle_d, float height, float height_d)
{
	float kp_a = 1.25; 
	float kp_h = 0.3;
	geometry_msgs::Twist twist;
	if (angle != 999)
	{
		twist.angular.z = kp_a*(angle - angle_d);
		twist.linear.x = -kp_h*(float)((height - height_d)/height_d);
	}
	else 
	{
		twist.angular.z = 0;
		twist.linear.x = 0;
	}
	std::cout << "Mesurements h alpha: " << height << " " << angle << std::endl;
	std::cout << "Command vx wz: " << twist.linear.x << " " << twist.angular.z << std::endl;
	myfile<< it_cnt<<" "<<clock()<<" " << twist.linear.x << " "<< twist.angular.z <<std::endl; // write
	pub.publish(twist);
}


void imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
	try {

	it_cnt++;
	//convert ROS image to CV image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

	//visp init...
	lineTraker(cv_ptr->image);

	// HOUGH transform
	float angle, lineEnd;
	lineDetector(cv_ptr->image, angle, lineEnd);
	twistPublisher(angle, 0, lineEnd, 420);
	
	//visualization source image


    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {

	h_min = 0;
	h_max = 10;
	lowThreshold = 0;
	it_cnt = 0; // iteration number

	//write time in filename
	time_t t = time(0);   // get time now
	struct tm * now = localtime( & t );
	char buffer [80];
	strftime (buffer,80," %Y%m%d_ %H%M",now);


	strcat(buffer,"result.txt");
	// write results in file
	myfile.open (buffer);
	myfile << "N Time vx wz \n";

    ros::init(argc, argv, "image_transport_subscriber");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_NAME, 1, imageCallBack);
	pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	ros::Rate loop_rate(10);


    //cv::namedWindow("source", CV_WINDOW_NORMAL);
    cv::namedWindow("hough", CV_WINDOW_NORMAL);
	cv::createTrackbar( "min Hue", "hough", &h_min, 180, on_trackbar_min );
	cv::createTrackbar( "max Hue", "hough", &h_max, 180, on_trackbar_max );
	cv::createTrackbar( "canny:", "hough", &lowThreshold, max_lowThreshold, on_trackbar_canny );

    ros::spin();
    cv::startWindowThread();

    cv::destroyWindow("source");
    cv::destroyWindow("hough");
    ros::shutdown();
    myfile.close();
    return 0;
}
