/*
 * PositionTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "PositionTask.h"

#include <stdexcept>


namespace Sai2Primitives
{


PositionTask::PositionTask(Sai2Model::Sai2Model* robot,
			const std::string link_name, 
			const Eigen::Affine3d control_frame,
			const double loop_time) :
	PositionTask(robot, link_name, control_frame.translation(), control_frame.linear(), loop_time) {}

PositionTask::PositionTask(Sai2Model::Sai2Model* robot, 
			const std::string link_name, 
			const Eigen::Vector3d pos_in_link, 
			const Eigen::Matrix3d rot_in_link,
			const double loop_time)
{

	Eigen::Affine3d control_frame = Eigen::Affine3d::Identity();
	control_frame.linear() = rot_in_link;
	control_frame.translation() = pos_in_link;

	_robot = robot;
	_link_name = link_name;
	_control_frame = control_frame;

	int dof = _robot->_dof;

	_robot->position(_current_position, _link_name, _control_frame.translation());
	_current_velocity.setZero();

	// default values for gains and velocity saturation
	_use_velocity_saturation_flag = false;
	_saturation_velocity = 0.3;
	_kp = 50.0;
	_kv = 14.0;
	_ki = 0.0;

	_kv_force = 10.0;

	// initialize matrices sizes
	_jacobian.setZero(3,dof);
	_projected_jacobian.setZero(3,dof);
	_Lambda.setZero(3,3);
	_Jbar.setZero(dof,3);
	_N.setZero(dof,dof);
	_N_prec = Eigen::MatrixXd::Identity(dof,dof);

	_sigma_motion = Matrix3d::Identity();
	_sigma_force = Matrix3d::Zero();

#ifdef USING_OTG 
	_use_interpolation_flag = true;

	_loop_time = loop_time;
	_otg = new OTG(_current_position, _loop_time);

	// default values for interpolation
	_otg->setMaxVelocity(0.3);
	_otg->setMaxAcceleration(1.0);
	_otg->setMaxJerk(3.0);
#endif

	reInitializeTask();
}

void PositionTask::reInitializeTask()
{
	int dof = _robot->_dof;

	_robot->position(_desired_position, _link_name, _control_frame.translation());
	_desired_velocity.setZero();
	_desired_acceleration.setZero();
	_desired_force.setZero();

	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;
	_step_desired_acceleration = _desired_acceleration;

	_integrated_position_error.setZero();

	_motion_control.setZero();
	_force_control.setZero();
	_task_force.setZero();
	_first_iteration = true;

#ifdef USING_OTG 
	_otg->reInitialize(_current_position);
#endif
}


void PositionTask::updateTaskModel(const Eigen::MatrixXd N_prec)
{
	if(N_prec.rows() != N_prec.cols())
	{
		throw std::invalid_argument("N_prec matrix not square in PositionTask::updateTaskModel\n");
	}
	if(N_prec.rows() != _robot->_dof)
	{
		throw std::invalid_argument("N_prec matrix size not consistent with robot dof in PositionTask::updateTaskModel\n");
	}

	_N_prec = N_prec;

	_robot->Jv(_jacobian, _link_name, _control_frame.translation());
	_projected_jacobian = _jacobian * _N_prec;
	_robot->operationalSpaceMatrices(_Lambda, _Jbar, _N, _projected_jacobian, _N_prec);
}


void PositionTask::computeTorques(Eigen::VectorXd& task_joint_torques)
{

	// get time since last call for the I term
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_t_curr = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
	}
	else
	{
		_t_curr = std::chrono::high_resolution_clock::now();
	}
	_t_diff = _t_curr - _t_prev;

	// update constroller state
	_robot->position(_current_position, _link_name, _control_frame.translation());
	_current_velocity = _projected_jacobian * _robot->_dq;
	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;
	_step_desired_acceleration = _desired_acceleration;

	// compute next state from trajectory generation
#ifdef USING_OTG
	if(_use_interpolation_flag)
	{
		_otg->setGoalPositionAndVelocity(_desired_position, _desired_velocity);
		_otg->computeNextState(_step_desired_position, _step_desired_velocity, _step_desired_acceleration);
	}
#endif

	// update integrated error for I term
	_integrated_position_error += (_current_position - _step_desired_position) * _t_diff.count();

	// compute task force
	if(_use_velocity_saturation_flag)
	{
		_step_desired_velocity = -_kp / _kv * (_current_position - _step_desired_position) - _ki/_kv * _integrated_position_error;
		if(_step_desired_velocity.norm() > _saturation_velocity)
		{
			_step_desired_velocity *= _saturation_velocity / _step_desired_velocity.norm();
		}
		_motion_control = _sigma_motion * (_step_desired_acceleration -_kv*(_current_velocity - _step_desired_velocity));
		// _task_force = _Lambda * (_step_desired_acceleration -_kv*(_current_velocity - _step_desired_velocity));
	}
	else
	{
		_motion_control = _sigma_motion * (_step_desired_acceleration -_kp*(_current_position - _step_desired_position) - _kv*(_current_velocity - _step_desired_velocity ) - _ki * _integrated_position_error);
		// _task_force = _Lambda*(_step_desired_acceleration -_kp*(_current_position - _step_desired_position) - _kv*(_current_velocity - _step_desired_velocity ) - _ki * _integrated_position_error);
	}

	_force_control = _sigma_force * (_desired_force - _kv_force * _current_velocity);

	// compute task torques
	_task_force = _Lambda * _motion_control + _force_control;
	task_joint_torques = _projected_jacobian.transpose()*_task_force;

	// update previous time
	_t_prev = _t_curr;
}


void PositionTask::setMotionAxis(const Vector3d motion_axis)
{
	Vector3d normalized_axis = motion_axis.normalized();

	_sigma_motion = normalized_axis * normalized_axis.transpose();
	_sigma_force = Matrix3d::Identity() - _sigma_motion;
}

void PositionTask::setForceAxis(const Vector3d force_axis)
{
	Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis * normalized_axis.transpose();
	_sigma_motion = Matrix3d::Identity() - _sigma_force;
}

void PositionTask::setFullMotionControl()
{
	_sigma_motion = Matrix3d::Identity();
	_sigma_force = Matrix3d::Zero();
}

void PositionTask::setFullForceControl()
{
	_sigma_force = Matrix3d::Identity();
	_sigma_motion = Matrix3d::Zero();
}


} /* namespace Sai2Primitives */

