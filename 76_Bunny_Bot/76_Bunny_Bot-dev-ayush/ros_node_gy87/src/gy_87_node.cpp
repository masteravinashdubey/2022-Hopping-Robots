// ROS libraries
#include "ros/ros.h"

// Sensor libraries
#include "GY87.h"

// Main function
int main(int argc, char **argv)
{
	// Init ROS node
	ROS_INFO("gy_87_node: Init ROS...");
	ros::init(argc, argv, "gy_87");
	ros::NodeHandle imu_node;
	ros::Rate loop_rate(100); // 100 Hz publish rate

	// Init breakout board
	GY87 gy = GY87(imu_node);

	ROS_INFO("gy_87_node: Running...");
	gy.calibrate();

	ROS_INFO("gy_87_node: Running after calibration...");
	while (ros::ok())
	{
		// Read out and publish sensor data
		gy.publish();
	
		// ROS
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

