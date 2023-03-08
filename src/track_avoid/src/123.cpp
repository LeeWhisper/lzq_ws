#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"

using namespace std;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "123");
	tf::Transformer listener;
	tf::StampedTransform transform;
	try
	{
		listener.lookupTransform("odom", "base_footprint", ros::Time(0), transform);
		cout << "transfrom: " << transform.frame_id_ << endl;
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
		// continue;
	}
	ros::spin();
	return 0;
}
