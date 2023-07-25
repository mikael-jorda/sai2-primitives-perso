/*
 * PartialJointTask.cpp
 *
 *      Author: William Chong 
 */

#include "PartialJointTask.h"

#include <stdexcept>


namespace Sai2Primitives
{

PartialJointTask::PartialJointTask(Sai2Model::Sai2Model* robot, 
                                        const std::vector<int> selection, 
                                        const double loop_time)
{
	_robot = robot;
    _active_joints = selection;
    int dof = _active_joints.size();

	// _current_position = _robot->_q(_active_joints);
	_current_position.setZero(dof);
	for (int i = 0; i < dof; ++i) {
		_current_position(i) = _robot->_q(_active_joints[i]);
	}
	_current_velocity.setZero(dof);

	// default values for gains and velocity saturation
	_kp = 50.0;
	_kv = 14.0;
	_ki = 0.0;
	_use_velocity_saturation_flag = false;
	_saturation_velocity = M_PI/3.0*Eigen::VectorXd::Ones(dof);

	_use_isotropic_gains = true;
	_kp_mat = Eigen::MatrixXd::Zero(dof,dof);
	_kv_mat = Eigen::MatrixXd::Zero(dof,dof);
	_ki_mat = Eigen::MatrixXd::Zero(dof,dof);

	// initialize matrices sizes
	_N_prec = Eigen::MatrixXd::Identity(dof,dof);
	_M_modified = Eigen::MatrixXd::Zero(dof,dof);
    _J = Eigen::MatrixXd::Zero(dof,robot->dof());
    for (int i = 0; i < dof; ++i) {
        _J(i, _active_joints[i]) = 1;
    }

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

void PartialJointTask::reInitializeTask()
{
    int dof = _active_joints.size();

	// _desired_position = _robot->_q(_active_joints);
	_desired_position.setZero(dof);
	for (int i = 0; i < dof; ++i) {
		_desired_position(i) = _robot->_q(_active_joints[i]);
	}
	_desired_velocity.setZero(dof);
	_desired_acceleration.setZero(dof);

	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;
	_step_desired_acceleration = _desired_acceleration;

	_task_force.setZero();
	_integrated_position_error.setZero(dof);
	_first_iteration = true;	

#ifdef USING_OTG 
	_otg->reInitialize(_current_position);
#endif
}

void PartialJointTask::setDynamicDecouplingFull()
{
	_dynamic_decoupling_type = FULL_DYNAMIC_DECOUPLING;
}

void PartialJointTask::setDynamicDecouplingBIE()
{
	_dynamic_decoupling_type = BOUNDED_INERTIA_ESTIMATES;
}

void PartialJointTask::setDynamicDecouplingNone()
{
	_dynamic_decoupling_type = IMPEDANCE;
}

void PartialJointTask::setNonIsotropicGains(const Eigen::VectorXd& kp, const Eigen::VectorXd& kv, const Eigen::VectorXd& ki)
{
	int dof = _active_joints.size();

	if(kp.size() != dof || kv.size() != dof || ki.size() != dof)
	{
		throw std::invalid_argument("size of gain vector inconsistent with number of robot joints in JointTask::useNonIsotropicGains\n");
	}

	_use_isotropic_gains = false;

	_kp_mat.setZero(dof,dof);
	_kv_mat.setZero(dof,dof);
	_ki_mat.setZero(dof,dof);

	for(int i=0 ; i<dof ; i++)
	{
		_kp_mat(i,i) = kp(i);
		_kv_mat(i,i) = kv(i);
		_ki_mat(i,i) = ki(i);
	}
}
	
void PartialJointTask::setIsotropicGains(const double kp, const double kv, const double ki)
{
	_use_isotropic_gains = true;

	_kp = kp;
	_kv = kv;
	_ki = ki;
}


void PartialJointTask::updateTaskModel(const Eigen::MatrixXd N_prec)
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
    _robot->nullspaceMatrix(_N, _J, _N_prec);
	
	switch(_dynamic_decoupling_type)
	{
		case FULL_DYNAMIC_DECOUPLING :
		{
			_M_modified = (_J * _robot->_M_inv * _J.transpose()).inverse();
			break;
		}

		case BOUNDED_INERTIA_ESTIMATES :
		{
			_M_modified = (_J * _robot->_M_inv * _J.transpose()).inverse();
			for(int i=0 ; i<_active_joints.size() ; i++)
			{
				if(_M_modified(i,i) < 0.1)
				{
					_M_modified(i,i) = 0.1;
				}
			}            
			break;
		}

		case IMPEDANCE :
		{
			_M_modified = Eigen::MatrixXd::Identity(_active_joints.size(), _active_joints.size());
			break;
		}

		default :
		{
			_M_modified = (_J * _robot->_M_inv * _J.transpose()).inverse();
			break;		
		}
	}
}


void PartialJointTask::computeTorques(Eigen::VectorXd& task_joint_torques)
{
	int dof = _active_joints.size();

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
		_kp_mat = _kp * Eigen::MatrixXd::Identity(dof,dof);
		_kv_mat = _kv * Eigen::MatrixXd::Identity(dof,dof);
		_ki_mat = _ki * Eigen::MatrixXd::Identity(dof,dof);
	}

	// update constroller state
	// _current_position = _robot->_q(_active_joints);
	// _current_velocity = _robot->_dq(_active_joints);
	for (int i = 0; i < dof; ++i) {
		_current_position(i) = _robot->_q(_active_joints[i]);
	}
	for (int i = 0; i < dof; ++i) {
		_current_velocity(i) = _robot->_dq(_active_joints[i]);
	}
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

	// compute error for I term
	_integrated_position_error += (_current_position - _step_desired_position) * _t_diff.count();

	// compute task force (with velocity saturation if asked)
	if(_use_velocity_saturation_flag)
	{
		_step_desired_velocity = -_kp_mat*_kv_mat.inverse() * (_current_position - _step_desired_position) - _ki_mat*_kv_mat.inverse() * _integrated_position_error;
		// for(int i=0; i<_robot->dof(); i++)
		for (int i = 0; i < dof; ++i) 
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
		_task_force = (_step_desired_acceleration -_kv_mat*(_current_velocity - _step_desired_velocity));
	}
	else
	{
		_task_force = (_step_desired_acceleration -_kp_mat*(_current_position - _step_desired_position) - _kv_mat * (_current_velocity - _step_desired_velocity) - _ki_mat * _integrated_position_error);
	}

	_task_force = _J.transpose() * _M_modified * _task_force;

	// compute task torques
	task_joint_torques = _N_prec.transpose() * _task_force;

	// update previous time
	_t_prev = _t_curr;
}

double PartialJointTask::squaredNormError()
{
	// return (_robot->_q(_active_joints) - _desired_position).norm();
	double err = 0;
	for (int i = 0; i < _active_joints.size(); ++i) {
		err += (_robot->_q(_active_joints[i]) - _desired_position(i)) * (_robot->_q(_active_joints[i]) - _desired_position(i));
	}
	return err;
}

} /* namespace Sai2Primitives */ 

