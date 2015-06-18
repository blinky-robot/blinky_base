#include <boost/chrono.hpp>
#include <boost/thread.hpp>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "blinky_hardware/blinky_hardware.h"

void controlThread(ros::Rate rate, blinky_hardware::BlinkyHardware *robot, controller_manager::ControllerManager *cm)
{
	boost::chrono::duration<double> elapsed_duration;
	boost::chrono::steady_clock::time_point last_time = boost::chrono::steady_clock::now();
	boost::chrono::steady_clock::time_point this_time;
	ros::Time this_time_ros;

	while (ros::ok())
	{
		this_time = boost::chrono::steady_clock::now();
		elapsed_duration = this_time - last_time;
		ros::Duration elapsed(elapsed_duration.count());
		last_time = this_time;
		this_time_ros = ros::Time::now();

		robot->read(this_time_ros, elapsed);
		cm->update(this_time_ros, elapsed);
		robot->write(this_time_ros, elapsed);
		rate.sleep();

		boost::this_thread::interruption_point();
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "blinky_hardware_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	blinky_hardware::BlinkyHardware robot(nh, nh_priv);
	controller_manager::ControllerManager cm(&robot);

	boost::thread control_thread(boost::bind(controlThread, ros::Rate(50), &robot, &cm));

	ros::spin();

	control_thread.join();

	return 0;
}
