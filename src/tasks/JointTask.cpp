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
	_desired_position = _robot->_q;

	_current_velocity.setZero(dof);
	_desired_velocity.setZero(dof);

	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;
	_saturation_velocity = M_PI/4.0*Eigen::VectorXd::Ones(dof);

	_kp = 50.0;
	_kv = 14.0;
	_ki = 0.0;

	_task_force.setZero(dof);
	_integrated_position_error.setZero(dof);

	_N_prec = Eigen::MatrixXd::Identity(dof,dof);

	_first_iteration = true;
	
#ifdef USING_OTG 
	_loop_time = loop_time;
	_otg = new OTG(_current_position, _loop_time);

	_otg->setMaxVelocity(M_PI/4);
	_otg->setMaxAcceleration(M_PI/2);
	_otg->setMaxJerk(M_PI);
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
	_current_position = _robot->_q;
	_current_velocity = _robot->_dq;
	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;

	// compute next state from trajectory generation
#ifdef USING_OTG
	if(_use_interpolation_flag)
	{
		_otg->setGoalPosition(_desired_position);
		_otg->computeNextState(_step_desired_position, _step_desired_velocity);
	}
#endif

	// compute error for I term
	_integrated_position_error += (_current_position - _step_desired_position) * _t_diff.count();

	// compute task force (with velocity saturation if asked)
	if(_use_velocity_saturation_flag)
	{
		_step_desired_velocity = -_kp/_kv * (_current_position - _step_desired_position) - _ki/_kv * _integrated_position_error;
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
		_task_force = _robot->_M * (-_kv*(_current_velocity - _step_desired_velocity));
	}
	else
	{
		_task_force = _robot->_M*(-_kp*(_current_position - _step_desired_position) - _kv * _current_velocity - _ki * _integrated_position_error);
	}

	// compute task torques
	task_joint_torques = _N_prec.transpose() * _task_force;

	// update previous time
	_t_prev = _t_curr;
}

void JointTask::reInitializeTask()
{
	int dof = _robot->_dof;

	_current_position = _robot->_q;
	_desired_position = _robot->_q;

	_current_velocity.setZero(dof);
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


} /* namespace Sai2Primitives */

