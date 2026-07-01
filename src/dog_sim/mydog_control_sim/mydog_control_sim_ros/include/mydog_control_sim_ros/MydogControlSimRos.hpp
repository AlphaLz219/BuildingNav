/*!
 * @file    MydogControlSimRos.hpp
 * @author  Guiyang Xin
 * @date    Jan, 2021
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/atomic.hpp>

#include <Eigen/Geometry>

#include <tf/transform_broadcaster.h>

#include <std_srvs/Empty.h>
#include <controller_manager_msgs/SwitchController.h>

#include <sensor_msgs/Joy.h>

#include <controller_manager_msgs/SwitchController.h>

#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16MultiArray.h>

#include <ros/package.h>
#include <memory>
#include <MNN/Interpreter.hpp>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace mydog_control_sim_ros {

class MydogControlSimRos {

public:
	/*! Constructor.
	 *
	 * @param nodeHandle the ROS node handle.
	 * @param averageCalculatorPtr the average calculator.
	 */
	explicit MydogControlSimRos(ros::NodeHandle &nodeHandle);

	//! Destructor.
	~MydogControlSimRos() = default;

	virtual bool update();
	void rlActionInference();
	void publishPositionRos();
	void copyResponseToRequest(const gazebo_msgs::GetPhysicsProperties &getPhys,
			gazebo_msgs::SetPhysicsProperties &setPhys);

private:
	// Callback function for state feedback
	void stateCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void baseStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
	void buttonStartCallback(const std_msgs::Bool::ConstPtr &msg);
	void buttonWalkCallback(const std_msgs::Bool::ConstPtr &msg);
	void buttonStandCallback(const std_msgs::Bool::ConstPtr &msg);
	void forwardBackCallback(const std_msgs::Float32::ConstPtr &msg);
	void leftRightCallback(const std_msgs::Float32::ConstPtr &msg);
	void yawCallback(const std_msgs::Float32::ConstPtr &msg);
	void setPIDGains(double p, double d);

	//! ROS node handle.
	ros::NodeHandle &nodeHandle_;

private:
	//! If true, the real robot is controlled and not a simulated.
	bool isRealRobot_;
	bool fixedBase_ = false;

	//! Time step between two control updates in seconds.
	double timeStep_;

	ros::Publisher actuatorCommandsPublisher_1;
	ros::Publisher actuatorCommandsPublisher_2;
	ros::Publisher actuatorCommandsPublisher_3;
	ros::Publisher actuatorCommandsPublisher_4;
	ros::Publisher actuatorCommandsPublisher_5;
	ros::Publisher actuatorCommandsPublisher_6;
	ros::Publisher actuatorCommandsPublisher_7;
	ros::Publisher actuatorCommandsPublisher_8;
	ros::Publisher actuatorCommandsPublisher_9;
	ros::Publisher actuatorCommandsPublisher_10;
	ros::Publisher actuatorCommandsPublisher_11;
	ros::Publisher actuatorCommandsPublisher_12;

	ros::Publisher baseAngularVelocityPublisher;
	ros::Publisher projectedGravityPublisher;
	ros::Publisher dofPosPublisher;
	ros::Publisher dofVelPublisher;
	ros::Publisher lastActionsPublisher;

	ros::Subscriber stateSubscriber_;
	ros::Subscriber baseStateSubscriber_;
	ros::Subscriber joystickSubscriber_;

	std_msgs::Float64 msg1;
	std_msgs::Float64 msg2;
	std_msgs::Float64 msg3;
	std_msgs::Float64 msg4;
	std_msgs::Float64 msg5;
	std_msgs::Float64 msg6;
	std_msgs::Float64 msg7;
	std_msgs::Float64 msg8;
	std_msgs::Float64 msg9;
	std_msgs::Float64 msg10;
	std_msgs::Float64 msg11;
	std_msgs::Float64 msg12;

	std_msgs::Float32MultiArray baseAngularVelocity_msg;
	std_msgs::Float32MultiArray projectedGravity_msg;
	std_msgs::Float32MultiArray dofPos_msg;
	std_msgs::Float32MultiArray dofVel_msg;
	std_msgs::Float32MultiArray lastActions_msg;

	ros::ServiceClient gazeboSrvGetPhysics_;
	ros::ServiceClient gazeboSrvSetPhysics_;

	gazebo_msgs::GetPhysicsProperties gazGetPhysics_;
	gazebo_msgs::SetPhysicsProperties gazSetPhysics_;

	double t = 0;

	int flag_ = 0;

	// subscriber message
	sensor_msgs::JointState jointState_;
	gazebo_msgs::ModelStates baseState_;

	Eigen::VectorXd q;
	Eigen::VectorXd v;

	bool pause_gazebo = true;

	int linearX_, linearY_;
	double x_scale_, y_scale_;

	Eigen::Matrix<double, 6, 1> desiredTwistInControlFrame_;
	Eigen::Quaterniond baseOrientationQuaternionInWorldFrame;

	bool enable_walking_ = false;
	bool switchBetweenStanceAndWalking_ = false;

	ros::Subscriber button_start_sub;
	ros::Subscriber button_walk_sub;
	ros::Subscriber button_stand_sub;
	ros::Subscriber forward_back_sub;
	ros::Subscriber left_right_sub;
	ros::Subscriber yaw_sub;
	bool tringer_ = false;

	bool start_btn = false;
	bool walk_btn = false;
	bool stand_btn = false;
	
	std::string rname_;

	// PID gains for position controllers
	double init_p_gain_ = 100.0;  // Initial high P gain for standing
	double init_d_gain_ = 5.0;    // Initial high D gain for standing
	double rl_p_gain_ = 40.0;     // RL P gain
	double rl_d_gain_ = 1.0;      // RL D gain
	bool pid_switched_ = false;   // Flag to track if PID has been switched
	bool init_pid_set_ = false;   // Flag to track if initial PID has been set
	ros::Timer init_pid_timer_;   // Timer for delayed initial PID setting
	void initPIDTimerCallback(const ros::TimerEvent& event);

	// rl
	ros::Publisher desiredJointVelocities_pub;
	std_msgs::Float32MultiArray desiredJointVelocities_msg;
	float rl_obs[45];
	float dof_vel_scale = 0.1;
	float actions[12] = {0.0f};
	float rl_actions[12] = {0.0f};
	float torques[12] = {0.0f};
	float cmd[3] = {0.0, 0.0, 0.0};
	float action_scale = 1.0;
	Eigen::Vector3d gravity_;
	int action_repeat = 1;
	int action_repeat_times = 4;

	// for Xue Yufei network
	// std::string network_path = ros::package::getPath("mydog_control_sim_ros") + std::string("/../../MNN/network/little_dog");
	// std::string lstm_path = network_path + std::string("/lstm.mnn");
	// std::string encoder_path = network_path + std::string("/encoder.mnn");
	// std::string mlp_path = network_path + std::string("/v_xueyf/net.mnn"); 
	// float mlp_in_arr[45] = {0};

	//MLP Network
	int obs_dim;
	MNN::Tensor* obs_temp_mnn = nullptr;
	MNN::Tensor* obs_mnn = nullptr;
	MNN::Tensor* act_mnn = nullptr;
	// MLP Network

	// for LSTM-based network
	std::string network_path = ros::package::getPath("mydog_control_sim_ros") + std::string("/../../MNN/network/little_dog/v9");
	std::string lstm_path = network_path + std::string("/lstm.mnn");
	std::string encoder_path = network_path + std::string("/encoder.mnn");
	std::string mlp_path = network_path + std::string("/mlp.mnn"); 
	std::string mlp_xue_path = network_path + std::string("/net.mnn"); 
	float mlp_in_arr[69] = {0};

	MNN::ScheduleConfig config;

	std::shared_ptr<MNN::Interpreter> lstm_net;
	std::shared_ptr<MNN::Interpreter> encoder_net;
	std::shared_ptr<MNN::Interpreter> mlp_net;
	std::shared_ptr<MNN::Interpreter> net_;

	MNN::Session* lstm_session;
	MNN::Session* encoder_session;
	MNN::Session* mlp_session;
	MNN::Session* session_;

	MNN::Tensor* lstm_input;
	MNN::Tensor* h0;
	MNN::Tensor* c0;

	MNN::Tensor* lstm_out;
	MNN::Tensor* hx;
	MNN::Tensor* cx;

	MNN::Tensor* lstm_input_temp;
	MNN::Tensor* h0_temp;
	MNN::Tensor* c0_temp;

	MNN::Tensor* encoder_input;
	MNN::Tensor* encoder_out;
	MNN::Tensor* encoder_out_temp;

	MNN::Tensor* mlp_input;
	MNN::Tensor* mlp_out;
	MNN::Tensor* mlp_input_temp;
	MNN::Tensor* mlp_out_temp;
};

} /* namespace locomotion_controller */
