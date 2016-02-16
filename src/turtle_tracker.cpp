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

int k; // iteration counter
// ros global
static const std::string TOPIC_NAME = "camera/rgb/image_raw";
ros::Publisher pub;
// visp global
vpImage<unsigned char> I_visp;
vpMeLine line;
vpMe me;


void trackerInit(cv::Mat I)
{
	std::cout << "trackerInit" << std::endl;
    vpImageConvert::convert(I, I_visp);

	#if defined(VISP_HAVE_X11)
		vpDisplayX d(I_visp, 0, 0, "Camera view");
	#elif defined(VISP_HAVE_GDI)
		vpDisplayGDI d(I_visp, 0, 0, "Camera view");
	#else
		std::cout << "No image viewer is available..." << std::endl;
	#endif


    vpDisplay::setTitle(I_visp, "My image");
    vpDisplay::display(I_visp);
    vpDisplay::flush(I_visp);

	me.setRange(25);
	me.setThreshold(15000);
	me.setSampleStep(10);

	line.setMe(&me);
	line.setDisplay(vpMeSite::RANGE_RESULT);
	line.initTracking(I_visp);
}

void trackerRun(cv::Mat I)
{
	std::cout << "trackerRun" << std::endl;

    vpImageConvert::convert(I, I_visp);
	std::cout << "convert" << std::endl;

	#if defined(VISP_HAVE_X11)
		vpDisplayX d(I_visp, 0, 0, "Camera view");
	#elif defined(VISP_HAVE_GDI)
		vpDisplayGDI d(I_visp, 0, 0, "Camera view");
	#else
		std::cout << "No image viewer is available..." << std::endl;
	#endif


    vpDisplay::setTitle(I_visp, "My image");
    vpDisplay::display(I_visp);
	std::cout << "display" << std::endl;

    line.track(I_visp);
	std::cout << "track" << std::endl;

    line.display(I_visp, vpColor::red);
	std::cout << "display" << std::endl;

    vpDisplay::flush(I_visp);
	std::cout << "flush" << std::endl;
	cv::waitKey(200);

}

void imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
	try {

	std::cout << "imageCallBack" << std::endl;

	//convert ROS image to CV image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

	if (k == 0) {trackerInit(cv_ptr->image);}
	else {trackerRun(cv_ptr->image);}

	k++;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {

	k =0;
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
