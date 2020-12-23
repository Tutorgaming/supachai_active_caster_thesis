#include "apt_handle/activecaster.h"
#include <thread>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using std::string;
using std::exception;
using std::cout;
using std::cin;
using std::endl;


int main(int argc, char** argv)
{
    // ROS Node Initialization 
    ros::init(argc, argv, "active_caster");
    ros::NodeHandle nodeHandle;

	ros::Publisher cmd_vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	// Class Initialization 
    ACTIVECASTER::SENSORACQUISITION sensorAcquisition;
	ACTIVECASTER::VIRTUALADMITTANCE virtualAdmittance;
	ACTIVECASTER::TIME robot_time;
	std::chrono::high_resolution_clock::time_point startTime;
	std::chrono::duration<double> loop;
	// Serial Initialization
	serial::Serial arduino_sensorAcquisition(
		"/dev/ttyACM1", 57600, serial::Timeout::simpleTimeout(100000));

    // Global Variables
	double mappedForce[3] = { 0, 0, 0 };
	double velRef_Robot[3] = { 0, 0, 0 };
	double virtual_vel_saturate[3] = {2, 2, 2};
	double virtual_vel_deadzoe[3] = {0.1, 0.1, 0.1};
	double innertia[3] = {1,1,1};
	double damping[3] ={1,1,1};

	double dT = 0;
	bool ret = true;
    // Debug Flag 
	bool debug_force = true;
	bool debug_velocity = true;

	// Init Serial 
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	arduino_sensorAcquisition.close();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	arduino_sensorAcquisition.open();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	if (!sensorAcquisition.isOpen(arduino_sensorAcquisition))
		return -1;

	if (!sensorAcquisition.force_calibration(arduino_sensorAcquisition))
		return -1;
    ROS_INFO("Sensor Initialization Complete !");

	// Setup classes
	virtualAdmittance.setVirtualAdmittanceParam(innertia, damping);
	virtualAdmittance.setOutputConstraint(virtual_vel_saturate, virtual_vel_deadzoe, velRef_Robot);
	robot_time.startTiming();

	while (ros::ok()) {
		
		dT = robot_time.timeLoop();

		if (!sensorAcquisition.forceRead(arduino_sensorAcquisition))
		{
		   ROS_WARN("Force Read Error ! - Skip this loop");
		   continue;
		}

		sensorAcquisition.forceMapping(mappedForce);
		virtualAdmittance.getVelAdmittance(dT, mappedForce, velRef_Robot);
		
		if(debug_force)
		{
		    ROS_INFO("MappedForce (%lf, %lf, %lf) ", 
							mappedForce[0], 
							mappedForce[1], 
							mappedForce[2]
		    );
		}

		if(debug_velocity)
		{
		    ROS_INFO("---------");
		    ROS_INFO("Vel X : %lf", velRef_Robot[0]);
		    ROS_INFO("Vel Z : %lf", velRef_Robot[2]);
		}

		geometry_msgs::Twist drive_cmd;
        drive_cmd.linear.x = velRef_Robot[0];
		drive_cmd.angular.z = velRef_Robot[2];
		cmd_vel_pub.publish(drive_cmd);
	}
    ROS_WARN("NODE SHUTDOWN !");
	return 1;
}

