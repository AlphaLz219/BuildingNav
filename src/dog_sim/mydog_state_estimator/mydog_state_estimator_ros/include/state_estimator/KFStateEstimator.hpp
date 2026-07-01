/*
 * KFStateEstimator.hpp
 *
 *  Created on: Mar 14, 2023
 *      Author: anymal
 */
#include <ros/ros.h>
#include <Eigen/Dense>

#ifndef INCLUDE_STATE_ESTIMATOR_KFSTATEESTIMATOR_HPP_
#define INCLUDE_STATE_ESTIMATOR_KFSTATEESTIMATOR_HPP_

class KFStateEstimator {
public:
	/*!
	 * Constructor.
	 */
	KFStateEstimator(double dt);
	void run(Eigen::Matrix3d qua_imu, Eigen::Vector3d free_acc_imu,
			Eigen::Matrix<double, 24, 1> foot_state_in_base_frame,
			Eigen::Vector3d omegaBody, Eigen::Vector4i contactEstimate,
			Eigen::Vector3d &base_in_world_frame,
			Eigen::Vector3d &base_velocity_in_world_frame,
			Eigen::Vector3d &base_velocity_in_base_frame);

protected:
	Eigen::Matrix<double, 18, 1> _xhat; //including p_b, v_b and p_f(foot positions), 3+3+12=18
	Eigen::Matrix<double, 12, 1> _ps;
	Eigen::Matrix<double, 12, 1> _vs;
	Eigen::Matrix<double, 18, 18> _A;
	Eigen::Matrix<double, 18, 18> _Q0;
	Eigen::Matrix<double, 18, 18> _P;
	Eigen::Matrix<double, 28, 28> _R0;
	Eigen::Matrix<double, 18, 3> _B;
	Eigen::Matrix<double, 28, 18> _C; //including p_f, v_f and foot height, 12+12+4=28
	double dt_;
	Eigen::Vector4i trusts_ = Eigen::Vector4i::Zero();
};

#endif /* INCLUDE_STATE_ESTIMATOR_KFSTATEESTIMATOR_HPP_ */
