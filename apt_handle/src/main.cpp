#include "apt_handle/activecaster.h"
#include <thread>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

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
    std::string port_name;
    nodeHandle.param<std::string>("port", port_name, "/dev/ttyACM0");

    ros::Publisher cmd_vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher vec_pub = nodeHandle.advertise<geometry_msgs::Vector3>("/force", 1);

    ROS_INFO("HELLO WORLD! ");
    // Class Initialization
    ACTIVECASTER::SENSORACQUISITION sensorAcquisition;
    ACTIVECASTER::VIRTUALADMITTANCE virtualAdmittance;
    ACTIVECASTER::TIME robot_time;
    std::chrono::high_resolution_clock::time_point startTime;
    std::chrono::duration<double> loop;
    // Serial Initialization
    serial::Serial arduino_sensorAcquisition(
        port_name, 57600, serial::Timeout::simpleTimeout(5000));

    ROS_INFO("Serial Connected !");
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
    ROS_INFO("Resetting Serial !");
    ros::Duration(0.5).sleep(); //std::this_thread::sleep_for(std::chrono::milliseconds(500));
    arduino_sensorAcquisition.close();
    ros::Duration(0.5).sleep();//std::this_thread::sleep_for(std::chrono::milliseconds(500));
    arduino_sensorAcquisition.open();
    ros::Duration(0.5).sleep();//std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ROS_INFO("Resetting Complete !");
    cout << arduino_sensorAcquisition.isOpen() << endl;

    if (!sensorAcquisition.isOpen(arduino_sensorAcquisition)){ //if(!arduino_sensorAcquisition.isOpen()){ //
        ROS_ERROR("Cannot Open Sensor Acquisition");
        return -1;
    }

    ROS_INFO("FORCE CABLIBRATIONING !");
    if (!sensorAcquisition.force_calibration(arduino_sensorAcquisition)){
        ROS_ERROR("Cannot Force Calibrate");
        return -1;
    }

    ROS_INFO("Sensor Initialization Complete !");
    // Setup classes
    virtualAdmittance.setVirtualAdmittanceParam(innertia, damping);
    virtualAdmittance.setOutputConstraint(virtual_vel_saturate, virtual_vel_deadzoe, velRef_Robot);
    robot_time.startTiming();
    double ros_dt;

    while (ros::ok()) {
        ros::Time start_ros_time = ros::Time::now();
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

        // Message Publishing
        geometry_msgs::Twist drive_cmd;
        drive_cmd.linear.x = velRef_Robot[0];
        drive_cmd.angular.z = velRef_Robot[2];
        cmd_vel_pub.publish(drive_cmd);

        // MappedForce
        geometry_msgs::Vector3 force;
        force.x = mappedForce[0];
        force.y = mappedForce[1];
        force.z = mappedForce[2];
        vec_pub.publish(force);

        // ROS Time Management
        ros::Time end_ros_time = ros::Time::now();
        ros_dt = (end_ros_time-start_ros_time).toSec();
        // Spinning
        ros::spinOnce();
    }
    ROS_WARN("NODE SHUTDOWN !");
    return 1;
}

