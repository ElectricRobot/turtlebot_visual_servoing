// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// cpp
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>


static const std::string TOPIC_NAME = "camera/rgb/image_raw";
static const std::string DEPTH_TOPIC_NAME = "camera/depth/image_raw";

int it_cnt; //iterator counter
std::ofstream myfile; // file where results will be writen


void topicCallback(const geometry_msgs::Twist& msg) {
	it_cnt++;
	myfile<< it_cnt<<" "<<clock()<<" " << msg.linear.x << " "<< msg.angular.z <<std::endl; // write

}



int main(int argc, char **argv) {

	it_cnt = 0; // iteration number
	std::clock_t start = clock();
	// write results in file
	myfile.open ("leader.txt");
	myfile << "N Time vx wz \n";

	ros::init(argc, argv, "read_topic");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000, topicCallback);
	ros::spin();

    ros::shutdown();
    myfile.close();
    return 0;
}
