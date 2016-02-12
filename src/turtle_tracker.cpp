// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
// cpp
#include <math.h>
#include <vector>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
/*#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
*/
// Visp
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpMeLine.h>



static const std::string TOPIC_NAME = "camera/rgb/image_raw";
ros::Publisher pub;

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
	line.initTracking(img);*/
}

void imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
	try {

	//convert ROS image to CV image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

	//visp init...
	lineTraker(cv_ptr->image);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "image_transport_subscriber");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_NAME, 1, imageCallBack);
	pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	ros::Rate loop_rate(10);

    //cv::namedWindow("source", CV_WINDOW_NORMAL);

    ros::spin();
    cv::startWindowThread();

    cv::destroyWindow("source");

    ros::shutdown();
    return 0;
}
