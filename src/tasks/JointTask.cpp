/*
 * JointTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "JointTask.h"

#include <stdexcept>


namespace Sai2Primitives
{


JointTask::JointTask(Sai2Model::Sai2Model* robot,
			const double loop_time)
{
	_robot = robot;
	int dof = _robot->_dof;

	_current_position = _robot->_q;
	_current_velocity.setZero(dof);

	// default values for gains and velocity saturation
	_kp = 50.0;
	_kv = 14.0;
	_ki = 0.0;
	_use_velocity_saturation_flag = false;
	_saturation_velocity = M_PI/3.0*Eigen::VectorXd::Ones(dof);

	_use_isotropic_gains = true;
	_kp_vec = _kp*Eigen::VectorXd::Ones(dof);
	_kv_vec = _kv*Eigen::VectorXd::Ones(dof);
	_ki_vec = _ki*Eigen::VectorXd::Ones(dof);

	_kp_mat = Eigen::MatrixXd::Zero(dof,dof);
	_kv_mat = Eigen::MatrixXd::Zero(dof,dof);
	_ki_mat = Eigen::MatrixXd::Zero(dof,dof);

	// initialize matrices sizes
	_N_prec = Eigen::MatrixXd::Identity(dof,dof);

#ifdef USING_OTG 
	_use_interpolation_flag = true;
	_loop_time = loop_time;
	_otg = new OTG(_current_position, _loop_time);

	_otg->setMaxVelocity(M_PI/3);
	_otg->setMaxAcceleration(M_PI);
	_otg->setMaxJerk(3*M_PI);
#endif
	reInitializeTask();
}

void JointTask::reInitializeTask()
{
	int dof = _robot->_dof;

	_desired_position = _robot->_q;
	_desired_velocity.setZero(dof);

	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;

	_task_force.setZero();
	_integrated_position_error.setZero(dof);
	_first_iteration = true;	

#ifdef USING_OTG 
	_otg->reInitialize(_current_position);
#endif
}

void JointTask::updateTaskModel(const Eigen::MatrixXd N_prec)
{
	if(N_prec.rows() != N_prec.cols())
	{
		throw std::invalid_argument("N_prec matrix not square in JointTask::updateTaskModel\n");
	}
	if(N_prec.rows() != _robot->_dof)
	{
		throw std::invalid_argument("N_prec matrix size not consistent with robot dof in JointTask::updateTaskModel\n");
	}

	_N_prec = N_prec;
}


void JointTask::computeTorques(Eigen::VectorXd& task_joint_torques)
{
	int dof = _robot->_dof;

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

	// update matrix gains
	if(_use_isotropic_gains)
	{
		_kp_vec = _kp*Eigen::VectorXd::Ones(dof);
		_kv_vec = _kv*Eigen::VectorXd::Ones(dof);
		_ki_vec = _ki*Eigen::VectorXd::Ones(dof);
	}
	for(int i=0 ; i<dof ; i++)
	{
		_kp_mat(i,i) = _kp_vec(i);
		_kv_mat(i,i) = _kv_vec(i);
		_ki_mat(i,i) = _ki_vec(i);
	}

	// update constroller state
	_current_position = _robot->_q;
	_current_velocity = _robot->_dq;
	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;

	// compute next state from trajectory generation
#ifdef USING_OTG
	if(_use_interpolation_flag)
	{
		_otg->setGoalPositionAndVelocity(_desired_position, _desired_velocity);
		_otg->computeNextState(_step_desired_position, _step_desired_velocity);
	}
#endif

	// compute error for I term
	_integrated_position_error += (_current_position - _step_desired_position) * _t_diff.count();

	// compute task force (with velocity saturation if asked)
	if(_use_velocity_saturation_flag)
	{
		_step_desired_velocity = -_kp_mat*_kv_mat.inverse() * (_current_position - _step_desired_position) - _ki_mat*_kv_mat.inverse() * _integrated_position_error;
		for(int i=0; i<_robot->dof(); i++)
		{
			if(_step_desired_velocity(i) > _saturation_velocity(i))
			{
				_step_desired_velocity(i) = _saturation_velocity(i);
			}
			else if(_step_desired_velocity(i) < -_saturation_velocity(i))
			{
				_step_desired_velocity(i) = -_saturation_velocity(i);
			}
		}
		_task_force = _robot->_M * (-_kv_mat*(_current_velocity - _step_desired_velocity));
	}
	else
	{
		_task_force = _robot->_M*(-_kp_mat*(_current_position - _step_desired_position) - _kv_mat * _current_velocity - _ki_mat * _integrated_position_error);
	}

	// compute task torques
	task_joint_torques = _N_prec.transpose() * _task_force;

	// update previous time
	_t_prev = _t_curr;
}

} /* namespace Sai2Primitives */

