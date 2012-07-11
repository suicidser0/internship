#include "ros/ros.h"
#include "internship/Num.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_private_name");

	ros::NodeHandle n;
	ros::NodeHandle nh1("~");
	ros::NodeHandle nh2("~foo");

	ros::Publisher pub = nh2.advertise<internship::Num>("private_name", 100);
	
	internship::Num a;
	a.age = 10;
	
	pub.publish(a);

	ros::spin();
	
	return 0;
}
