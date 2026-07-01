#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

ros::Time current_time, last_time;
geometry_msgs::Quaternion odom_quat;
geometry_msgs::TransformStamped odom_trans;
nav_msgs::Odometry odom_;
void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
	int index = 1;
	auto it = std::find(msg->name.begin(), msg->name.end(), "mydog");
	if (it != msg->name.end()) {
		index = distance(msg->name.begin(), it);
	}
	current_time = ros::Time::now();
	odom_quat = msg->pose[index].orientation;

	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base";

	odom_trans.transform.translation.x = msg->pose[index].position.x;
	odom_trans.transform.translation.y = msg->pose[index].position.y;
	odom_trans.transform.translation.z = msg->pose[index].position.z;
	odom_trans.transform.rotation = odom_quat;

	//set the position
	odom_.pose.pose.position.x = msg->pose[index].position.x;
	odom_.pose.pose.position.y = msg->pose[index].position.y;
	odom_.pose.pose.position.z = msg->pose[index].position.z;
	odom_.pose.pose.orientation = odom_quat;

	//set the velocity
	odom_.child_frame_id = "base";
//	 odom_.twist.twist.linear.x = msg->twist[1].linear.x;
//	 odom_.twist.twist.linear.y = msg->twist[1].linear.y;
//	 odom_.twist.twist.angular.z = msg->twist[1].angular.z;
	odom_.twist.twist = msg->twist[index];
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 400);
	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber sub = n.subscribe("/gazebo/model_states", 400,
			gazeboCallback);

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.1;
	double vy = -0.1;
	double vth = 0.1;

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(200.0);

	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base";

	odom_.pose.pose.position.x = 0;
	odom_.pose.pose.position.y = 0;
	odom_.pose.pose.position.z = 1.2;
	odom_.pose.pose.orientation.w = 1;
	odom_.pose.pose.orientation.x = 0;
	odom_.pose.pose.orientation.y = 0;
	odom_.pose.pose.orientation.z = 0;

	odom_trans.transform.translation.x = 0;
	odom_trans.transform.translation.y = 0;
	odom_trans.transform.translation.z = 1.2;
	odom_trans.transform.rotation.w = 1;
	odom_trans.transform.rotation.x = 0;
	odom_trans.transform.rotation.y = 0;
	odom_trans.transform.rotation.z = 0;

	while (n.ok()) {

		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		//compute odometry in a typical way given the velocities of the robot
//    double dt = 0.0025;
//    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
//    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
//    double delta_th = vth * dt;
//
//    x += delta_x;
//    y += delta_y;
//    th += delta_th;
//
//    //since all odometry is 6DOF we'll need a quaternion created from yaw
//    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//first, we'll publish the transform over tf

		odom_trans.header.stamp = current_time;

//    odom_trans.transform.translation.x = x;
//    odom_trans.transform.translation.y = y;
//    odom_trans.transform.translation.z = 0.0;
//    odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS

		odom_.header.stamp = current_time;
		odom_.header.frame_id = "odom";

		//publish the message
		odom_pub.publish(odom_);

		last_time = current_time;
		r.sleep();
	}
}
