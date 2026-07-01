/*!
 * @file    mydog_highlevel_controller_node.cpp
 * @author  Guiyang Xin
 * @date    Jan, 2020
 */

#include "mydog_control_sim_ros/MydogControlSimRos.hpp"

#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>

void copyResponseToRequest(const gazebo_msgs::GetPhysicsProperties &getPhys,
		gazebo_msgs::SetPhysicsProperties &setPhys) {
	setPhys.request.gravity = getPhys.response.gravity;
	setPhys.request.max_update_rate = getPhys.response.max_update_rate;
	setPhys.request.ode_config = getPhys.response.ode_config;
	setPhys.request.time_step = getPhys.response.time_step;
}

int main(int argc, char **argv) {

	// Initialize ROS.
	ros::init(argc, argv, "mydog_control_sim_ros_node");
	ros::NodeHandle nodeHandle("~");

	// Create a mydog control sim ROS node.
	mydog_control_sim_ros::MydogControlSimRos mydogControlSimRos(nodeHandle);

	// Waiting for loading ros stuff
	std::chrono::seconds dura(3);
	std::this_thread::sleep_for(dura);

	// Start gazebo simulation
	ros::ServiceClient pauseGazebo;
	ros::ServiceClient unpauseGazebo;
	ros::ServiceClient gazeboSrvGetPhysics_;
	ros::ServiceClient gazeboSrvSetPhysics_;

	std_srvs::Empty pauseSrv;
	std_srvs::Empty unpauseSrv;
	gazebo_msgs::GetPhysicsProperties gazGetPhysics_;
	gazebo_msgs::SetPhysicsProperties gazSetPhysics_;

	pauseGazebo = nodeHandle.serviceClient<std_srvs::Empty>(
			"/gazebo/pause_physics");
	unpauseGazebo = nodeHandle.serviceClient<std_srvs::Empty>(
			"/gazebo/unpause_physics");

	gazeboSrvGetPhysics_ = nodeHandle.serviceClient<
			gazebo_msgs::GetPhysicsProperties>(
			"/gazebo/get_physics_properties");
	gazeboSrvSetPhysics_ = nodeHandle.serviceClient<
			gazebo_msgs::SetPhysicsProperties>(
			"/gazebo/set_physics_properties");

	if (gazeboSrvGetPhysics_.call(gazGetPhysics_)) {
		copyResponseToRequest(gazGetPhysics_, gazSetPhysics_);
//		gazSetPhysics_.request.max_update_rate = 100;
		gazSetPhysics_.request.gravity.z = -0.1;
		gazeboSrvSetPhysics_.call(gazSetPhysics_);
	}

	unpauseGazebo.call(unpauseSrv);

	// Define running frequency
	double dt = 0.005;
	nodeHandle.param<double>(ros::this_node::getName() + "/time_step",
				dt, 0.005);
	// ros::Rate loop_rate(1/dt);
	ros::Rate loop_rate(50);

	// Cycle loop
	while (ros::ok()) {
		mydogControlSimRos.update();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
