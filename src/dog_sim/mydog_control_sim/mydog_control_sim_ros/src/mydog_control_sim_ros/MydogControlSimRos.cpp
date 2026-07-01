/*!
 * @file    MydogControlSimRos.cpp
 * @author  Guiyang Xin
 * @date    Jan, 2021
 */

#include "mydog_control_sim_ros/MydogControlSimRos.hpp"
#include <chrono>

#include <unistd.h>

namespace mydog_control_sim_ros
{

	MydogControlSimRos::MydogControlSimRos(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
	{

#ifndef NDEBUG
		// Print a warning if built in debug.
		ROS_INFO_STREAM(
			"CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

		//	 Ros parameters
		nodeHandle_.param<double>(ros::this_node::getName() + "/time_step",
								  timeStep_, 0.005);
		nodeHandle_.param<bool>(ros::this_node::getName() + "/isRealRobot",
								isRealRobot_, false);
		nodeHandle_.param<bool>(ros::this_node::getName() + "/fixedBase",
								fixedBase_, false);
		nodeHandle_.param<int>(ros::this_node::getName() + "/linearX", linearX_, 1);
		nodeHandle_.param<int>(ros::this_node::getName() + "/linearY", linearY_, 1);
		nodeHandle_.param<double>(ros::this_node::getName() + "/x_scale", x_scale_,
								  1);
		nodeHandle_.param<double>(ros::this_node::getName() + "/y_scale", y_scale_,
								  1);
		nodeHandle_.param<std::string>(ros::this_node::getName() + "/rname", rname_,
									   "ask_3");

		ROS_INFO_STREAM(
			"[MydogControlSimRos] Is on real robot: " << (isRealRobot_ ? "yes" : "no"));

		// Ros publishers
		actuatorCommandsPublisher_1 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint1_position_controller/command", 30);
		actuatorCommandsPublisher_2 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint2_position_controller/command", 30);
		actuatorCommandsPublisher_3 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint3_position_controller/command", 30);
		actuatorCommandsPublisher_4 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint4_position_controller/command", 30);
		actuatorCommandsPublisher_5 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint5_position_controller/command", 30);
		actuatorCommandsPublisher_6 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint6_position_controller/command", 30);
		actuatorCommandsPublisher_7 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint7_position_controller/command", 30);
		actuatorCommandsPublisher_8 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint8_position_controller/command", 30);
		actuatorCommandsPublisher_9 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint9_position_controller/command", 30);
		actuatorCommandsPublisher_10 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint10_position_controller/command", 30);
		actuatorCommandsPublisher_11 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint11_position_controller/command", 30);
		actuatorCommandsPublisher_12 = nodeHandle_.advertise<std_msgs::Float64>(
			"/" + rname_ + "/joint12_position_controller/command", 30);

		baseAngularVelocityPublisher = nodeHandle_.advertise<std_msgs::Float32MultiArray>(
			"/" + rname_ + "/base_ang_vel", 30);
		projectedGravityPublisher = nodeHandle_.advertise<std_msgs::Float32MultiArray>(
			"/" + rname_ + "/projected_gravity", 30);
		dofPosPublisher = nodeHandle_.advertise<std_msgs::Float32MultiArray>(
			"/" + rname_ + "/dof_pos", 30);
		dofVelPublisher = nodeHandle_.advertise<std_msgs::Float32MultiArray>(
			"/" + rname_ + "/dof_vel", 30);
		lastActionsPublisher = nodeHandle_.advertise<std_msgs::Float32MultiArray>(
			"/" + rname_ + "/last_actions", 30);

		// Ros subscribers
		stateSubscriber_ = nodeHandle_.subscribe("/" + rname_ + "/joint_states", 30,
												 &MydogControlSimRos::stateCallback, this);
		baseStateSubscriber_ = nodeHandle_.subscribe("/gazebo/model_states", 30,
													 &MydogControlSimRos::baseStateCallback, this);
		joystickSubscriber_ = nodeHandle_.subscribe("/joy", 30,
													&MydogControlSimRos::joyCallback, this);

		button_start_sub = nodeHandle_.subscribe("/ask/dog/start", 30, &MydogControlSimRos::buttonStartCallback, this);
		button_walk_sub = nodeHandle_.subscribe("/ask/dog/walk", 30, &MydogControlSimRos::buttonWalkCallback, this);
		button_stand_sub = nodeHandle_.subscribe("/ask/dog/stand", 30, &MydogControlSimRos::buttonStandCallback, this);
		left_right_sub = nodeHandle_.subscribe("/ask/dog/left_right", 30, &MydogControlSimRos::leftRightCallback, this);
		forward_back_sub = nodeHandle_.subscribe("/ask/dog/forward_back", 30, &MydogControlSimRos::forwardBackCallback, this);
		yaw_sub = nodeHandle_.subscribe("/ask/dog/yaw", 30, &MydogControlSimRos::yawCallback, this);

		if (fixedBase_)
		{
			q.resize(12, 1);
			q.setZero();
			v.resize(12, 1);
			v.setZero();
		}
		else
		{
			q.resize(19, 1);
			q.setZero();
			q(6) = 1;
			v.resize(18, 1);
			v.setZero();
		}
		desiredTwistInControlFrame_.setZero();
		baseOrientationQuaternionInWorldFrame.x() = 0;
		baseOrientationQuaternionInWorldFrame.y() = 0;
		baseOrientationQuaternionInWorldFrame.z() = 0;
		baseOrientationQuaternionInWorldFrame.w() = 1;

		// rl_obs publish msg
		baseAngularVelocity_msg.data = {0, 0, 0};
		projectedGravity_msg.data = {0, 0, 0};
		dofPos_msg.data =
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		dofVel_msg.data =
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		lastActions_msg.data =
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

		// rl
		desiredJointVelocities_pub = nodeHandle_.advertise<std_msgs::Float32MultiArray>("/mydog/rl_desired_joint_velocities", 30);
		gravity_ << 0.0, 0.0, -1.0;

		// MLP Network Initialization
		mlp_net = std::shared_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(mlp_xue_path.c_str()));
		mlp_session = mlp_net->createSession(config);
		obs_mnn = mlp_net->getSessionInput(mlp_session, "obs");
		act_mnn = mlp_net->getSessionOutput(mlp_session, "act");
		obs_dim = obs_mnn->shape().back();
		obs_temp_mnn = MNN::Tensor::create<float>(obs_mnn->shape(), NULL, MNN::Tensor::CAFFE);
		// Initialize Finish

		desiredJointVelocities_msg.data.resize(12);
		for (int i = 0; i < 12; i++)
		{
			desiredJointVelocities_msg.data[i] = 0.0;
		}

		// Set initial high PID gains for standing with a delay (wait for controllers to start)
		init_pid_timer_ = nodeHandle_.createTimer(ros::Duration(2.0), &MydogControlSimRos::initPIDTimerCallback, this, true);  // one-shot timer
		ROS_INFO_STREAM("[MydogControlSimRos] Will set initial PID gains in 2 seconds...");
	}

	bool MydogControlSimRos::update()
	{
		t = t + timeStep_;
		if (t > 3 || flag_ == 1)
		{
			gazeboSrvGetPhysics_ = nodeHandle_.serviceClient<
				gazebo_msgs::GetPhysicsProperties>(
				"/gazebo/get_physics_properties");
			gazeboSrvSetPhysics_ = nodeHandle_.serviceClient<
				gazebo_msgs::SetPhysicsProperties>(
				"/gazebo/set_physics_properties");

			if (gazeboSrvGetPhysics_.call(gazGetPhysics_))
			{
				copyResponseToRequest(gazGetPhysics_, gazSetPhysics_);
				gazSetPhysics_.request.max_update_rate = 1000;
				gazSetPhysics_.request.gravity.z = -9.81;
				gazeboSrvSetPhysics_.call(gazSetPhysics_);
			}
			if (enable_walking_)
			{
				// if (action_repeat % action_repeat_times == 0)
				// {
				// 	action_repeat = 1;
					rlActionInference();
					desiredJointVelocities_pub.publish(desiredJointVelocities_msg);
				// }
				// else
				// {
				// 	action_repeat++;
				// }
				// rlActionInference();

				// publish actions
				msg1.data = actions[0];
				msg2.data = actions[1];
				msg3.data = actions[2];
				msg4.data = actions[3];
				msg5.data = actions[4];
				msg6.data = actions[5];
				msg7.data = actions[6];
				msg8.data = actions[7];
				msg9.data = actions[8];
				msg10.data = actions[9];
				msg11.data = actions[10];
				msg12.data = actions[11];

				t = 0;
				flag_ = 1;

				publishPositionRos();
			}
		}
		else
		{
			msg1.data = 0.0;
			msg2.data = 0.0;
			msg3.data = 0.0;
			msg4.data = 0.0;
			msg5.data = 0.0;
			msg6.data = 0.0;
			msg7.data = 0.0;
			msg8.data = 0.0;
			msg9.data = 0.0;
			msg10.data = 0.0;
			msg11.data = 0.0;
			msg12.data = 0.0;

			publishPositionRos();
		}

		return true;
	}

	void MydogControlSimRos::publishPositionRos()
	{
		actuatorCommandsPublisher_1.publish(msg1);
		actuatorCommandsPublisher_2.publish(msg2);
		actuatorCommandsPublisher_3.publish(msg3);
		actuatorCommandsPublisher_4.publish(msg4);
		actuatorCommandsPublisher_5.publish(msg5);
		actuatorCommandsPublisher_6.publish(msg6);
		actuatorCommandsPublisher_7.publish(msg7);
		actuatorCommandsPublisher_8.publish(msg8);
		actuatorCommandsPublisher_9.publish(msg9);
		actuatorCommandsPublisher_10.publish(msg10);
		actuatorCommandsPublisher_11.publish(msg11);
		actuatorCommandsPublisher_12.publish(msg12);
	}

	void MydogControlSimRos::stateCallback(
		const sensor_msgs::JointState::ConstPtr &msg)
	{
		jointState_ = *msg;
		if (fixedBase_)
		{
			for (int i = 0; i < 12; i++)
			{
				q(i) = jointState_.position[i];
				v(i) = jointState_.velocity[i];
			}
			//
		}
		else
		{
			for (int i = 0; i < 12; i++)
			{
				q(i + 7) = jointState_.position[i];
				v(i + 6) = jointState_.velocity[i];
			}
		}
	}

	void MydogControlSimRos::baseStateCallback(
		const gazebo_msgs::ModelStates::ConstPtr &msg)
	{
		baseState_ = *msg;
		int index = 1;
		if (fixedBase_)
		{
		}
		else
		{
			auto it = std::find(baseState_.name.begin(), baseState_.name.end(),
								"mydog");
			if (it != baseState_.name.end())
			{
				index = distance(baseState_.name.begin(), it);
			}

			q(0) = baseState_.pose[index].position.x;
			q(1) = baseState_.pose[index].position.y;
			q(2) = baseState_.pose[index].position.z;
			q(3) = baseState_.pose[index].orientation.x;
			q(4) = baseState_.pose[index].orientation.y;
			q(5) = baseState_.pose[index].orientation.z;
			q(6) = baseState_.pose[index].orientation.w;

			v(0) = baseState_.twist[index].linear.x;
			v(1) = baseState_.twist[index].linear.y;
			v(2) = baseState_.twist[index].linear.z;
			v(3) = baseState_.twist[index].angular.x;
			v(4) = baseState_.twist[index].angular.y;
			v(5) = baseState_.twist[index].angular.z;
		}
	}

	void MydogControlSimRos::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
	{

		cmd[0] = x_scale_ * joy->axes[linearX_];
		cmd[1] = y_scale_ * joy->axes[linearY_];
		cmd[2] = joy->axes[3];

		if (joy->buttons[0])
		{
			enable_walking_ = true;
		}
	}

	void MydogControlSimRos::copyResponseToRequest(
		const gazebo_msgs::GetPhysicsProperties &getPhys,
		gazebo_msgs::SetPhysicsProperties &setPhys)
	{
		setPhys.request.gravity = getPhys.response.gravity;
		setPhys.request.max_update_rate = getPhys.response.max_update_rate;
		setPhys.request.ode_config = getPhys.response.ode_config;
		setPhys.request.time_step = getPhys.response.time_step;
	}

	void MydogControlSimRos::leftRightCallback(const std_msgs::Float32::ConstPtr &msg)
	{
		desiredTwistInControlFrame_(0) = 0.0;
		cmd[1] = y_scale_ * msg->data;
		desiredTwistInControlFrame_(2) = 0.0;
		desiredTwistInControlFrame_(5) = 0.0;
	}

	void MydogControlSimRos::forwardBackCallback(const std_msgs::Float32::ConstPtr &msg)
	{
		cmd[0] = x_scale_ * msg->data;
		desiredTwistInControlFrame_(1) = 0.0;
		desiredTwistInControlFrame_(2) = 0.0;
		desiredTwistInControlFrame_(5) = 0.0;
	}

	void MydogControlSimRos::yawCallback(const std_msgs::Float32::ConstPtr &msg)
	{
		cmd[2] = msg->data;
	}

	void MydogControlSimRos::buttonStartCallback(const std_msgs::Bool::ConstPtr &msg)
	{
		start_btn = msg->data;
		if (start_btn == true)
		{
			// Switch to RL PID gains when starting
			if (!pid_switched_)
			{
				setPIDGains(rl_p_gain_, rl_d_gain_);
				pid_switched_ = true;
				ROS_INFO_STREAM("[MydogControlSimRos] Switched to RL PID gains: P=" << rl_p_gain_ << ", D=" << rl_d_gain_);
			}
			enable_walking_ = true;
		}
		start_btn = false;
	}

	void MydogControlSimRos::buttonWalkCallback(const std_msgs::Bool::ConstPtr &msg)
	{
		walk_btn = msg->data;
		if (walk_btn = true)
		{
			switchBetweenStanceAndWalking_ = true;
		}
		walk_btn = false;
	}

	void MydogControlSimRos::buttonStandCallback(const std_msgs::Bool::ConstPtr &msg)
	{
		start_btn = msg->data;
		if (start_btn = true)
		{
			switchBetweenStanceAndWalking_ = false;
		}
		stand_btn = false;
	}

	void MydogControlSimRos::rlActionInference()
	{
		// ros::Time all_start_time = ros::Time::now();
		baseOrientationQuaternionInWorldFrame.x() = q(3);
		baseOrientationQuaternionInWorldFrame.y() = q(4);
		baseOrientationQuaternionInWorldFrame.z() = q(5);
		baseOrientationQuaternionInWorldFrame.w() = q(6);
		// base_ang_vel (in base frame)
		Eigen::Vector3d v_temp = baseOrientationQuaternionInWorldFrame.toRotationMatrix().inverse() * v.segment<3>(3);
		rl_obs[0] = v_temp(0);
		rl_obs[1] = v_temp(1);
		rl_obs[2] = v_temp(2);
		// projected_gravity
		Eigen::Vector3d gravity_project = baseOrientationQuaternionInWorldFrame.toRotationMatrix().inverse() * gravity_;
		rl_obs[3] = gravity_project(0);
		rl_obs[4] = gravity_project(1);
		rl_obs[5] = gravity_project(2);
		// dof_pos
		rl_obs[6] = q(7);
		rl_obs[7] = q(8);
		rl_obs[8] = q(9);
		rl_obs[9] = q(10);
		rl_obs[10] = q(11);
		rl_obs[11] = q(12);
		rl_obs[12] = q(13);
		rl_obs[13] = q(14);
		rl_obs[14] = q(15);
		rl_obs[15] = q(16);
		rl_obs[16] = q(17);
		rl_obs[17] = q(18);
		// dof_vel * obs_scales.dof_vel(0.1)
		rl_obs[18] = v(6) * dof_vel_scale;
		rl_obs[19] = v(7) * dof_vel_scale;
		rl_obs[20] = v(8) * dof_vel_scale;
		rl_obs[21] = v(9) * dof_vel_scale;
		rl_obs[22] = v(10) * dof_vel_scale;
		rl_obs[23] = v(11) * dof_vel_scale;
		rl_obs[24] = v(12) * dof_vel_scale;
		rl_obs[25] = v(13) * dof_vel_scale;
		rl_obs[26] = v(14) * dof_vel_scale;
		rl_obs[27] = v(15) * dof_vel_scale;
		rl_obs[28] = v(16) * dof_vel_scale;
		rl_obs[29] = v(17) * dof_vel_scale;
		// last_actions
		rl_obs[30] = actions[0];
		rl_obs[31] = actions[1];
		rl_obs[32] = actions[2];
		rl_obs[33] = actions[3];
		rl_obs[34] = actions[4];
		rl_obs[35] = actions[5];
		rl_obs[36] = actions[6];
		rl_obs[37] = actions[7];
		rl_obs[38] = actions[8];
		rl_obs[39] = actions[9];
		rl_obs[40] = actions[10];
		rl_obs[41] = actions[11];

		// concatenate command
		for (int i = 0; i < 3; i++)
		{
			rl_obs[42 + i] = cmd[i];
		}

		// publish rl_obs msg
		for (int i = 0; i < 3; i++)
		{
			baseAngularVelocity_msg.data[i] = rl_obs[i];
		}
		for (int i = 3; i < 6; i++)
		{
			projectedGravity_msg.data[i - 3] = rl_obs[i];
		}
		for (int i = 6; i < 18; i++)
		{
			dofPos_msg.data[i - 6] = rl_obs[i];
		}
		for (int i = 18; i < 30; i++)
		{
			dofVel_msg.data[i - 18] = rl_obs[i];
		}
		for (int i = 30; i < 42; i++)
		{
			lastActions_msg.data[i - 30] = rl_obs[i];
		}
		baseAngularVelocityPublisher.publish(baseAngularVelocity_msg);
		projectedGravityPublisher.publish(projectedGravity_msg);
		dofPosPublisher.publish(dofPos_msg);
		dofVelPublisher.publish(dofVel_msg);
		lastActionsPublisher.publish(lastActions_msg);

		// MLP Network Inference
		memcpy(obs_temp_mnn->host<float>() + (obs_temp_mnn->shape()[1] - 1) * obs_dim, rl_obs, obs_dim * sizeof(float));
		obs_mnn->copyFromHostTensor(obs_temp_mnn);

		mlp_net->runSession(mlp_session);
		memcpy(rl_actions, act_mnn->host<float>(), act_mnn->size());

		// 平移时序观测值
		for (int i = 0; i < obs_mnn->shape()[1] - 1; i++)
		{
			memcpy(obs_temp_mnn->host<float>() + i * obs_dim, obs_temp_mnn->host<float>() + (i + 1) * obs_dim,
				obs_dim * sizeof(float));
		}
		// MLP Network Inference Finish

		for (int i = 0; i < 12; i++)
		{
			rl_actions[i] = std::max(-1.0f, std::min(1.0f, rl_actions[i]));
			rl_actions[i] *= action_scale;
			// std::cout << "actions[" << i << "]:" << actions[i] << "\t";
		}

		// calculate desired joint velocities
		for (int i = 0; i < 12; i++)
		{
			desiredJointVelocities_msg.data[i] = (rl_actions[i] - actions[i]) / (action_repeat_times * timeStep_);
			actions[i] = rl_actions[i];
		}
		// ros::Time all_end_time = ros::Time::now();
		// std::cout << "all_duration: " << all_end_time - all_start_time << std::endl;
	}

	void MydogControlSimRos::initPIDTimerCallback(const ros::TimerEvent& event)
	{
		if (!init_pid_set_)
		{
			setPIDGains(init_p_gain_, init_d_gain_);
			init_pid_set_ = true;
			ROS_INFO_STREAM("[MydogControlSimRos] Initial PID gains set: P=" << init_p_gain_ << ", D=" << init_d_gain_);
		}
	}

	void MydogControlSimRos::setPIDGains(double p, double d)
	{
		// Use dynamic_reconfigure to set PID gains for all joint position controllers
		// Service path: /<robot_name>/joint<N>_position_controller/pid/set_parameters
		for (int i = 1; i <= 12; i++)
		{
			std::string service_name = "/" + rname_ + "/joint" + std::to_string(i) + "_position_controller/pid/set_parameters";
			
			dynamic_reconfigure::ReconfigureRequest srv_req;
			dynamic_reconfigure::ReconfigureResponse srv_resp;
			
			dynamic_reconfigure::DoubleParameter p_param;
			p_param.name = "p";
			p_param.value = p;
			srv_req.config.doubles.push_back(p_param);
			
			dynamic_reconfigure::DoubleParameter d_param;
			d_param.name = "d";
			d_param.value = d;
			srv_req.config.doubles.push_back(d_param);
			
			dynamic_reconfigure::DoubleParameter i_param;
			i_param.name = "i";
			i_param.value = 0.0;
			srv_req.config.doubles.push_back(i_param);
			
			if (ros::service::call(service_name, srv_req, srv_resp))
			{
				ROS_DEBUG_STREAM("Set PID for joint" << i << ": P=" << p << ", D=" << d);
			}
			else
			{
				ROS_WARN_STREAM("Failed to set PID for joint" << i);
			}
		}
	}

} /* namespace locomotion_controller */
