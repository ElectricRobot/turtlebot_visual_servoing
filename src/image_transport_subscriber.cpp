#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <vector>

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
bool init;
ros::Publisher pub;


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

void lineParam(cv::Rect roi, std::vector<cv::Vec4i> lines, int &p1x, int &p1y, int &p2x, int &p2y)
{
	int p1x_acc, p1y_acc, p2x_acc, p2y_acc;
	p1x_acc=0; p1y_acc=0; p2x_acc=0; p2y_acc=0;
	int k = 0;

	// calculate average line
	for( size_t i = 0; i < lines.size(); i++ )
	{
		cv::Vec4i l = lines[i];
		if (l[0] < l[2] && l[0] < 30) //select vertical lines that start on image top
		{
			p1x_acc = p1x_acc + l[0] + roi.x; p1y_acc = p1y_acc+ l[1] + roi.y; p2x_acc = p2x_acc + l[2] + roi.x; p2y_acc = p2y_acc+ l[3]+roi.y;
			cv::line(mapDraw, cv::Point (l[0] + roi.x,l[1] + roi.y), cv::Point(l[2] + roi.x,l[3] + roi.y), cv::Scalar(0, 255, 0), 2, 8); //draw line
			k++;
		}
	}
	if (k != 0){p1x = p1x_acc/k; p1y = p1y_acc/k; p2x = p2x_acc/k; p2y = p2y_acc/k;}
}

void lineROI(int &roi_k, std::vector<std::vector<int> > &linePoints, std::vector<float> &lineAngles, cv::Mat img, cv::Mat img_draw)
{
	cv::Rect roi; //roi where line will be detected
	int dh, dw;	// roi size
	dw = 120; dh = 120;

	//create roi
	if (roi_k == 0)
	{
		dw = 120; dh = 320;
		roi = cv::Rect(0, 0.5*img.rows-0.5*dh, dw, dh); // the first roi is a fixed window
		cv::rectangle(img_draw, roi, cv::Scalar( 0, 55, 255 ), +1, 4 ); // draw roi
		std::cout<<"roi0"<<std::endl;
	}
	else
	{

		for(int i = 0; i < linePoints.size(); i++){
			std::cout<<"Line points aaa: "<<linePoints[i][0]<<" "<<linePoints[i][1]<<" "<<linePoints[i][2]<<" "<<linePoints[i][3]<<" "<<std::endl;
		}
		std::cout<<"roi_k and linePoints.size()"<<roi_k<<" "<<linePoints.size()<<std::endl;

		if(linePoints[roi_k-1][3] + 0.5*dh > img.rows) {std::cout<<"if1"<<std::endl;roi = cv::Rect(linePoints[roi_k-1][2], linePoints[roi_k-1][3], dw, img.rows - linePoints[roi_k-1][3]); 		}
		else if(linePoints[roi_k-1][3] - 0.5*dh < 0) {std::cout<<"if2"<<std::endl;roi = cv::Rect(linePoints[roi_k-1][2], 0, dw, dh);}
		else {std::cout<<"if3"<<std::endl;roi = cv::Rect(linePoints[roi_k-1][2], linePoints[roi_k-1][3] - 0.5*dh, dw, dh);}
		std::cout<<"roi1 draw"<<std::endl;
		cv::rectangle(mapDraw, roi, cv::Scalar( 0, 255, 0 ), +1, 4 );
		std::cout<<"roi1 draw"<<std::endl;
	}

	// line detection by Hough algorithm
	std::vector<cv::Vec4i> lines; // create vector for line points storage
	int p1x, p1y, p2x, p2y; // line points
		std::cout<<"hough"<<std::endl;
	HoughLinesP(img(roi), lines, 1, CV_PI/180, 50, 50, 20 ); 
		std::cout<<"hough"<<std::endl;

	float alpha = 999; //if no line is detected, alpha=999
	std::vector<int> linerow;
	if (lines.size() > 0) 
	{
		std::cout<<"lparams"<<std::endl;
		lineParam(roi, lines, p1x, p1y, p2x, p2y); // select the calculate line points average
		std::cout<<"lparams"<<std::endl;
		linerow.push_back(p1x); linerow.push_back(p1y); linerow.push_back(p2x); linerow.push_back(p2y);
		linePoints.push_back(linerow);
		alpha =  atan((float)(p2y-p1y)/(p2x-p1x)); // calculate angle with vertical
		lineAngles.push_back(alpha);
		std::cout<<"Angle"<<roi_k<<": "<< alpha*(180/3.1416) <<std::endl; //print angle
		cv::line(mapDraw, cv::Point (p1x,p1y), cv::Point(p2x,p2y), cv::Scalar(0, 0, 255), 2, 8); //draw mean line
	}
	else{roi_k = 4;}// stop seeking for line... no line was found in current roi


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

void houghTransform(cv::Mat img, float &alpha)
{

	// rotate image needed for hough detection
	img = rotateImg(img, 90);
	// canny edge detection
	cv::Mat imgMap;
	cannyDetector(img, imgMap); // retrives a binary image with contours of orange spots
	cvtColor(imgMap, mapDraw, CV_GRAY2BGR); // color image for drawing

	// parameters for line detection
	int roi_k; // roi counter (from 1 to 4)
	std::vector<std::vector<int> > linePoints; // point coordinates of the segemented line
	std::vector<float> lineAngles; // anlges of the segemented line
	
	roi_k = 0;
	while(roi_k < 4)
	{
		lineROI(roi_k, linePoints, lineAngles, imgMap, mapDraw);
		roi_k++;
	std::cout<<roi_k<<std::endl;
	}

	std::cout<<"Printing lines angles ending points... "<<std::endl;
	for(int i = 0; i < linePoints.size(); i++){
		std::cout<<"Line "<<i<<": "<<linePoints[i][2]<<" "<<linePoints[i][3]<<" "<<std::endl;
		std::cout<<"Angle "<<i<<": "<<lineAngles[i]<<std::endl;
	}


/*
		lines.clear();
		HoughLinesP(imgMap(roi2), lines, 1, CV_PI/180, 50, 50, 20 );
		p1x = 0; p1y = 0; p2x = 0; p2y = 0;
		if (lines.size() > 0) 
		{
			lineParam(roi2, lines, p1x, p1y, p2x, p2y);
			std::cout<<"Angle2: "<< atan((float)(p2y-p1y)/(p2x-p1x))*(180/3.1416) <<std::endl;
			//alpha =  atan((float)(p2y-p1y)/(p2x-p1x));
			cv::line(mapDraw, cv::Point(p1x,p1y), cv::Point(p2x,p2y), cv::Scalar(0, 255, 0), 1.5, 8); //draw line
		}
		else alpha = 999;
	}

	// detect line ROI3
	if(alpha != 999) // then detect the second piece of the line
	{
		cv::Rect roi3 ;
		dw = 120; // height constant 120px
		//dh = 240*(sin(alpha)/cos(alpha)); // new width is varible in relation to alpha
		dh = 120;

		if(p2y + 0.5*dh > img.rows) {roi3 = cv::Rect(p2x, p2y, dw, img.rows - p2y);}
		else if(p2y - 0.5*dh < 0) {roi3 = cv::Rect(p2x, 0, dw, dh);}
		else {roi3 = cv::Rect(p2x, p2y-0.5*dh, dw, dh);}

		cv::rectangle(mapDraw, roi3, cv::Scalar( 0, 255, 0 ), +1, 4 );
		lines.clear();
		HoughLinesP(imgMap(roi3), lines, 1, CV_PI/180, 50, 50, 20 );
		p1x = 0; p1y = 0; p2x = 0; p2y = 0;
		if (lines.size() > 0) 
		{
			lineParam(roi3, lines, p1x, p1y, p2x, p2y);
			std::cout<<"Angle3: "<< atan((float)(p2y-p1y)/(p2x-p1x))*(180/3.1416) <<std::endl;
			//alpha =  atan((float)(p2y-p1y)/(p2x-p1x));
			cv::line(mapDraw, cv::Point(p1x,p1y), cv::Point(p2x,p2y), cv::Scalar(0, 255, 0), 1.5, 8); //draw line
		}
		else alpha = 999;
	}

	// detect line ROI4
	if(alpha != 999) // then detect the second piece of the line
	{
		cv::Rect roi4 ;
		dw = 120; // height constant 120px
		//dh = 240*(sin(alpha)/cos(alpha)); // new width is varible in relation to alpha
		dh = 120;

		if(p2y + 0.5*dh > img.rows) {roi4 = cv::Rect(p2x, p2y, dw, img.rows - p2y);}
		else if(p2y - 0.5*dh < 0) {roi4 = cv::Rect(p2x, 0, dw, dh);}
		else {roi4 = cv::Rect(p2x, p2y-0.5*dh, dw, dh);}
		cv::rectangle(mapDraw, roi4, cv::Scalar( 0, 255, 0 ), +1, 4 );
		lines.clear();
		HoughLinesP(imgMap(roi4), lines, 1, CV_PI/180, 50, 50, 20 );
		p1x = 0; p1y = 0; p2x = 0; p2y = 0;
		if (lines.size() > 0) 
		{
			lineParam(roi4, lines, p1x, p1y, p2x, p2y);
			if (p2x-p1x > 0) 
			{
				//alpha =  atan((float)(p2y-p1y)/(p2x-p1x));
				cv::line(mapDraw, cv::Point(p1x,p1y), cv::Point(p2x,p2y), cv::Scalar(0, 255, 0), 1.5, 8); //draw line
			}
			else alpha = 999;
		}
		else {alpha = 999;}
		std::cout<<"Angle4: "<< alpha*(180/3.1416) <<std::endl;
	}
	std::cout<<"line bottom: "<< p2y <<std::endl;
*/

	//result visualization
	mapDraw = rotateImg(mapDraw, -90);
	cv::imshow("hough", mapDraw);
	
}

/*void lineTraker(cv::Mat img)
{
	vpMe me;
	me.setRange(25);
	me.setThreshold(15000);
	me.setSampleStep(10);
	vpMeLine line;
	line.set Me(&me);
	line.set Display(vpMeSite::RANGE_RESULT);
	line.initTracking(img);
}
*/

void twistPublisher(float angle)
{
	if (angle != 999)
	{
		geometry_msgs::Twist twist;
		twist.angular.z = angle;
		pub.publish(twist);
	}
}


void imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
	try {

	//convert ROS image to CV image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

	/*//visp init...
	if (init == true)
	{
		#if defined(VISP_HAVE_X11)
	  	vpDisplayX d(cv_ptr->image, 0, 0, "Camera view");
		#else
	  	vpDisplayGDI d(cv_ptr->image, 0, 0, "Camera view");
		#endif
	}*/

	// HOUGH transform
	float angle;
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

void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Save image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "8UC1");
        mat = cv_ptr->image;
		
        //cv::Mat beConvertedMat(mat.rows, mat.cols, CV_8UC4, mat.data); // provide different view of m1 data
		imgsat = cv::Mat::zeros( mat.size(), mat.type() );

		// contrast
		cv::minMaxLoc(mat, &min, &max, NULL,  NULL);
		for( int r = 0; r < mat.rows ; r++ )
		{
			for( int c = 0; c < mat.cols ; c++ )
			{ 

				imgsat.at<uchar>(r,c) = 255 - 255*((mat.at<uchar>(r,c) - min)/ (max - min));
			}
		}			

        // Show image
        cv::imshow("depthview", imgsat); 
        cv::waitKey(10);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to '8UC1'.",
                msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {

	h_min = 0;
	h_max = 10;
	lowThreshold = 0;
	init = true;

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
