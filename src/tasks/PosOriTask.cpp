/*
 * PosOriTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "PosOriTask.h"

#include <stdexcept>


namespace Sai2Primitives
{


PosOriTask::PosOriTask(Sai2Model::Sai2Model* robot, 
			const std::string link_name, 
			const Eigen::Affine3d control_frame,
			const double loop_time) :
	PosOriTask(robot, link_name, control_frame.translation(), control_frame.linear(), loop_time) {}

PosOriTask::PosOriTask(Sai2Model::Sai2Model* robot, 
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

	_T_control_to_sensor = Eigen::Affine3d::Identity();  

	// motion
	_robot->position(_current_position, _link_name, _control_frame.translation());
	_robot->rotation(_current_orientation, _link_name);

	// default values for gains and velocity saturation
	_kp_pos = 50.0;
	_kv_pos = 14.0;
	_ki_pos = 0.0;
	_kp_ori = 50.0;
	_kv_ori = 14.0;
	_ki_ori = 0.0;

	_use_isotropic_gains = true;
	_kp_pos_vec = _kp_pos * Eigen::Vector3d::Ones();
	_kv_pos_vec = _kv_pos * Eigen::Vector3d::Ones();
	_ki_pos_vec = _ki_pos * Eigen::Vector3d::Ones();
	_kp_ori_vec = _kp_ori * Eigen::Vector3d::Ones();
	_kv_ori_vec = _kv_ori * Eigen::Vector3d::Ones();
	_ki_ori_vec = _ki_ori * Eigen::Vector3d::Ones();

	_kp_pos_mat = Eigen::Matrix3d::Zero();
	_kv_pos_mat = Eigen::Matrix3d::Zero();
	_ki_pos_mat = Eigen::Matrix3d::Zero();
	_kp_ori_mat = Eigen::Matrix3d::Zero();
	_kv_ori_mat = Eigen::Matrix3d::Zero();
	_ki_ori_mat = Eigen::Matrix3d::Zero();

	_kp_force = 1.0;
	_kv_force = 10.0;
	_ki_force = 0.7;
	_kp_moment = 1.0;
	_kv_moment = 10.0;
	_ki_moment = 0.7;

	_use_velocity_saturation_flag = false;
	_linear_saturation_velocity = 0.3;
	_angular_saturation_velocity = M_PI/3;

	// initialize matrices sizes
	_jacobian.setZero(6,dof);
	_projected_jacobian.setZero(6,dof);
	_Lambda.setZero(6,6);
	_Jbar.setZero(dof,6);
	_N.setZero(dof,dof);
	_N_prec = Eigen::MatrixXd::Identity(dof,dof);

	_first_iteration = true;

#ifdef USING_OTG
	_use_interpolation_flag = true; 
	_loop_time = loop_time;
	_otg = new OTG_posori(_current_position, _current_orientation, _loop_time);

	_otg->setMaxLinearVelocity(0.3);
	_otg->setMaxLinearAcceleration(1.0);
	_otg->setMaxLinearJerk(3.0);

	_otg->setMaxAngularVelocity(M_PI/3);
	_otg->setMaxAngularAcceleration(M_PI);
	_otg->setMaxAngularJerk(3*M_PI);
#endif
	reInitializeTask();
}

void PosOriTask::reInitializeTask()
{
	int dof = _robot->_dof;

	// motion
	_robot->position(_current_position, _link_name, _control_frame.translation());
	_robot->position(_desired_position, _link_name, _control_frame.translation());
	_robot->rotation(_current_orientation, _link_name);
	_robot->rotation(_desired_orientation, _link_name);

	_current_velocity.setZero();
	_desired_velocity.setZero();
	_current_angular_velocity.setZero();
	_desired_angular_velocity.setZero();

	_orientation_error.setZero();
	_integrated_position_error.setZero();
	_integrated_orientation_error.setZero();

	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;
	_step_desired_orientation = _desired_orientation;
	_step_orientation_error.setZero(3);
	_step_desired_angular_velocity.setZero(3);

	_sigma_position = Eigen::Matrix3d::Identity();
	_sigma_orientation = Eigen::Matrix3d::Identity();

	_desired_force.setZero();
	_sensed_force.setZero();
	_desired_moment.setZero();
	_sensed_moment.setZero();

	_integrated_force_error.setZero();
	_integrated_moment_error.setZero();

	_sigma_force.setZero();
	_sigma_moment.setZero();

	_closed_loop_force_control = false;
	_closed_loop_moment_control = false;

	_passivity_enabled = true;
	_passivity_observer = 0;
	_Rc_inv = 1.0;

	_task_force.setZero(6);
	_unit_mass_force.setZero(6);
	_first_iteration = true;	

#ifdef USING_OTG 
	_otg->reInitialize(_current_position, _current_orientation);
#endif
}

void PosOriTask::updateTaskModel(const Eigen::MatrixXd N_prec)
{
	if(N_prec.rows() != N_prec.cols())
	{
		throw std::invalid_argument("N_prec matrix not square in PosOriTask::updateTaskModel\n");
	}
	if(N_prec.rows() != _robot->_dof)
	{
		throw std::invalid_argument("N_prec matrix size not consistent with robot dof in PosOriTask::updateTaskModel\n");
	}

	_N_prec = N_prec;

	_robot->J_0(_jacobian, _link_name, _control_frame.translation());
	_projected_jacobian = _jacobian * _N_prec;
	_robot->operationalSpaceMatrices(_Lambda, _Jbar, _N, _projected_jacobian, _N_prec);

}


void PosOriTask::computeTorques(Eigen::VectorXd& task_joint_torques)
{

	// get time since last call for the I term
	_t_curr = std::chrono::high_resolution_clock::now();
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
	}
	_t_diff = _t_curr - _t_prev;

	// update matrix gains
	if(_use_isotropic_gains)
	{
		_kp_pos_vec = _kp_pos * Eigen::Vector3d::Ones();
		_kv_pos_vec = _kv_pos * Eigen::Vector3d::Ones();
		_ki_pos_vec = _ki_pos * Eigen::Vector3d::Ones();
		_kp_ori_vec = _kp_ori * Eigen::Vector3d::Ones();
		_kv_ori_vec = _kv_ori * Eigen::Vector3d::Ones();
		_ki_ori_vec = _ki_ori * Eigen::Vector3d::Ones();
	}
	for(int i=0 ; i<3 ; i++)
	{
		_kp_pos_mat(i,i) = _kp_pos_vec(i);
		_kv_pos_mat(i,i) = _kv_pos_vec(i);
		_ki_pos_mat(i,i) = _ki_pos_vec(i);
		_kp_ori_mat(i,i) = _kp_ori_vec(i);
		_kv_ori_mat(i,i) = _kv_ori_vec(i);
		_ki_ori_mat(i,i) = _ki_ori_vec(i);
	}

	Eigen::Vector3d force_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d position_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d moment_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d orientation_related_force = Eigen::Vector3d::Zero();

	// update controller state
	_robot->position(_current_position, _link_name, _control_frame.translation());
	_robot->rotation(_current_orientation, _link_name);
	_current_orientation = _current_orientation * _control_frame.linear(); // orientation of compliant frame in robot frame
	Sai2Model::orientationError(_orientation_error, _desired_orientation, _current_orientation);
	_current_velocity = _projected_jacobian.block(0,0,3,_robot->_dof) * _robot->_dq;
	_current_angular_velocity = _projected_jacobian.block(3,0,3,_robot->_dof) * _robot->_dq;

	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;
	_step_desired_orientation = _desired_orientation;
	_step_orientation_error = _orientation_error;
	_step_desired_angular_velocity = _desired_angular_velocity;

	// force related terms
	if(_closed_loop_force_control)
	{
		// update the integrated error
		_integrated_force_error += (_sensed_force - _desired_force) * _t_diff.count();

		// compute the feedback term
		Eigen::Vector3d force_feedback_term = - _kp_force * (_sensed_force - _desired_force) - _ki_force * _integrated_force_error; // - _kv_force * _current_velocity;

		// implement passivity observer and controller
		if(_passivity_enabled)
		{
			Eigen::Vector3d f_diff = _desired_force - _sensed_force;
			_passivity_observer +=  ((double)(f_diff.transpose() * _sigma_force * force_feedback_term) - 
					(double) (force_feedback_term.transpose() * _sigma_force * force_feedback_term) * _Rc_inv) * _t_diff.count();
			// _passivity_observer +=  ((double)(f_diff.transpose() * _sigma_force * force_feedback_term)) * _t_diff.count();

			std::cout << "PO : " << _passivity_observer << std::endl; // "\t" << _sensed_force(2) << "\t" << force_feedback_term(2) << std::endl << std::endl;
			std::cout << "Rc inv : " << _Rc_inv << std::endl; // "\t" << _sensed_force(2) << "\t" << force_feedback_term(2) << std::endl << std::endl;
			std::cout << "f_sensed: " << _sensed_force(2) << std::endl; // "\t" << _sensed_force(2) << "\t" << force_feedback_term(2) << std::endl << std::endl;
			// if(_passivity_observer < 0)
			// {
				double Rcb_inv = _Rc_inv + _passivity_observer / ((double) (force_feedback_term.transpose() * _sigma_force * force_feedback_term) * _t_diff.count());
				if(Rcb_inv > 1)
				{
					Rcb_inv = 1;
				}
				if(Rcb_inv < 0)
				{
					Rcb_inv = 0;
				}
				// std::cout << "Rcbis : " << 1.0 / Rcb_inv << std::endl;
				_passivity_observer += (double) (force_feedback_term.transpose() * _sigma_force * force_feedback_term) * (_Rc_inv - Rcb_inv) * _t_diff.count();
				_Rc_inv = Rcb_inv;
			// }


		}

		// compute the final contribution
		force_related_force = _sigma_force * (_desired_force + force_feedback_term * _Rc_inv - _kv_force * _current_velocity);
	}
	else
	{
		force_related_force = _sigma_force * _desired_force;
	}

	// moment related terms
	if(_closed_loop_moment_control)
	{
		// update the integrated error
		_integrated_moment_error += (_sensed_moment - _desired_moment) * _t_diff.count();

		// compute the feedback term
		Eigen::Vector3d moment_feedback_term = - _kp_moment * (_sensed_moment - _desired_moment) - _ki_moment * _integrated_moment_error - _kv_moment * _current_angular_velocity;

		// compute the final contribution
		moment_related_force = _sigma_moment * (_desired_moment + moment_feedback_term);
	}
	else
	{
		moment_related_force = _sigma_moment * _desired_moment;
	}

	// motion related terms
	// compute next state from trajectory generation
#ifdef USING_OTG
	if(_use_interpolation_flag)
	{
		_otg->setGoalPositionAndLinearVelocity(_desired_position, _desired_velocity);
		_otg->setGoalOrientationAndAngularVelocity(_desired_orientation, _current_orientation, _desired_angular_velocity);
		_otg->computeNextState(_step_desired_position, _step_desired_velocity,
							_step_desired_orientation, _step_desired_angular_velocity);
		Sai2Model::orientationError(_step_orientation_error, _step_desired_orientation, _current_orientation);
	}
#endif
	
	// linear motion
	// update integrated error for I term
	_integrated_position_error += (_current_position - _step_desired_position) * _t_diff.count();

	// final contribution
	if(_use_velocity_saturation_flag)
	{
		_step_desired_velocity = -_kp_pos_mat * _kv_pos_mat.inverse() * (_current_position - _step_desired_position) - _ki_pos_mat * _kv_pos_mat.inverse() * _integrated_position_error;
		if(_step_desired_velocity.norm() > _linear_saturation_velocity)
		{
			_step_desired_velocity *= _linear_saturation_velocity/_step_desired_velocity.norm();
		}
		position_related_force = _sigma_position * (-_kv_pos_mat*(_current_velocity - _step_desired_velocity));
	}
	else
	{
		position_related_force = _sigma_position*(-_kp_pos_mat*(_current_position - _step_desired_position) - _kv_pos_mat*(_current_velocity - _step_desired_velocity ) - _ki_pos_mat * _integrated_position_error);
	}


	// angular motion
	// update integrated error for I term
	_integrated_orientation_error += _step_orientation_error * _t_diff.count();

	// final contribution
	if(_use_velocity_saturation_flag)
	{
		_step_desired_angular_velocity = -_kp_ori_mat * _kv_ori_mat.inverse() * _step_orientation_error - _ki_ori_mat * _kv_ori_mat.inverse() * _integrated_orientation_error;
		if(_step_desired_angular_velocity.norm() > _angular_saturation_velocity)
		{
			_step_desired_angular_velocity *= _angular_saturation_velocity/_step_desired_angular_velocity.norm();
		}
		orientation_related_force = _sigma_orientation * (-_kv_ori_mat*(_current_angular_velocity - _step_desired_angular_velocity));
	}
	else
	{
		orientation_related_force = _sigma_orientation * ( -_kp_ori_mat*_step_orientation_error - _kv_ori_mat*(_current_angular_velocity - _step_desired_angular_velocity) - _ki_ori_mat*_integrated_orientation_error);
	}

	// compute task force
	Eigen::VectorXd force_moment_contribution(6), position_orientation_contribution(6);
	force_moment_contribution.head(3) = force_related_force;
	force_moment_contribution.tail(3) = moment_related_force;

	position_orientation_contribution.head(3) = position_related_force;
	position_orientation_contribution.tail(3) = orientation_related_force;

	_unit_mass_force = position_orientation_contribution;

	_task_force = _Lambda * position_orientation_contribution + force_moment_contribution;

	// compute task torques
	task_joint_torques = _projected_jacobian.transpose()*_task_force;

	// update previous time
	_t_prev = _t_curr;
}

bool PosOriTask::goalPositionReached(const double tolerance, const bool verbose)
{
	double position_error = (_desired_position - _current_position).transpose() * (_sigma_position) * (_desired_position - _current_position);
	position_error = sqrt(position_error);
	bool goal_reached = position_error < tolerance;
	if(verbose)
	{
		std::cout << "position error in PosOriTask : " << position_error << std::endl;
		std::cout << "Tolerance : " << tolerance << std::endl;
		std::cout << "Goal reached : " << goal_reached << std::endl << std::endl;
	}

	return goal_reached;
}

bool PosOriTask::goalOrientationReached(const double tolerance, const bool verbose)
{
	double orientation_error = _orientation_error.transpose() * _sigma_orientation * _orientation_error;
	orientation_error = sqrt(orientation_error);
	bool goal_reached = orientation_error < tolerance;
	if(verbose)
	{
		std::cout << "orientation error in PosOriTask : " << orientation_error << std::endl;
		std::cout << "Tolerance : " << tolerance << std::endl;
		std::cout << "Goal reached : " << goal_reached << std::endl << std::endl;
	}

	return goal_reached;
}

void PosOriTask::setForceSensorFrame(const std::string link_name, const Eigen::Affine3d transformation_in_link)
{
	if(link_name != _link_name)
	{
		throw std::invalid_argument("The link to which is attached the sensor should be the same as the link to which is attached the control frame in PosOriTask::setForceSensorFrame\n");
	}
	_T_control_to_sensor = _control_frame.inverse() * transformation_in_link;
}

void PosOriTask::updateSensedForceAndMoment(const Eigen::Vector3d sensed_force_sensor_frame, 
							    		    const Eigen::Vector3d sensed_moment_sensor_frame)
{
	// find the transform from base frame to control frame
	Eigen::Affine3d T_base_link;
	_robot->transform(T_base_link, _link_name);
	Eigen::Affine3d T_base_control = T_base_link * _control_frame;

	// find the resolved sensed force and moment in control frame
	_sensed_force = _T_control_to_sensor.rotation() * sensed_force_sensor_frame;
	_sensed_moment = _T_control_to_sensor.translation().cross(_sensed_force) + _T_control_to_sensor.rotation() * sensed_moment_sensor_frame;

	// rotate the quantities in base frame
	_sensed_force = T_base_control.rotation() * _sensed_force;
	_sensed_moment = T_base_control.rotation() * _sensed_moment;
}

void PosOriTask::setForceAxis(const Eigen::Vector3d force_axis)
{
	Eigen::Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis*normalized_axis.transpose();
	_sigma_position = Eigen::Matrix3d::Identity() - _sigma_force;

	resetIntegratorsLinear();
}

void PosOriTask::updateForceAxis(const Eigen::Vector3d force_axis)
{
	Eigen::Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis*normalized_axis.transpose();
	_sigma_position = Eigen::Matrix3d::Identity() - _sigma_force;
}

void PosOriTask::setLinearMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_position = normalized_axis*normalized_axis.transpose();
	_sigma_force = Eigen::Matrix3d::Identity() - _sigma_position;

	resetIntegratorsLinear();
}

void PosOriTask::updateLinearMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_position = normalized_axis*normalized_axis.transpose();
	_sigma_force = Eigen::Matrix3d::Identity() - _sigma_position;	
}

void PosOriTask::setFullForceControl()
{
	_sigma_force = Eigen::Matrix3d::Identity();
	_sigma_position.setZero();

	resetIntegratorsLinear();
}

void PosOriTask::setFullLinearMotionControl()
{
	_sigma_position = Eigen::Matrix3d::Identity();
	_sigma_force.setZero();

	resetIntegratorsLinear();
}

void PosOriTask::setMomentAxis(const Eigen::Vector3d moment_axis)
{
	Eigen::Vector3d normalized_axis = moment_axis.normalized();

	_sigma_moment = normalized_axis*normalized_axis.transpose();
	_sigma_orientation = Eigen::Matrix3d::Identity() - _sigma_moment;

	resetIntegratorsAngular();
}

void PosOriTask::updateMomentAxis(const Eigen::Vector3d moment_axis)
{
	Eigen::Vector3d normalized_axis = moment_axis.normalized();

	_sigma_moment = normalized_axis*normalized_axis.transpose();
	_sigma_orientation = Eigen::Matrix3d::Identity() - _sigma_moment;	
}

void PosOriTask::setAngularMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_orientation = normalized_axis*normalized_axis.transpose();
	_sigma_moment = Eigen::Matrix3d::Identity() - _sigma_orientation;

	resetIntegratorsAngular();
}

void PosOriTask::updateAngularMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_orientation = normalized_axis*normalized_axis.transpose();
	_sigma_moment = Eigen::Matrix3d::Identity() - _sigma_orientation;
}

void PosOriTask::setFullMomentControl()
{
	_sigma_moment = Eigen::Matrix3d::Identity();
	_sigma_orientation.setZero();

	resetIntegratorsAngular();
}

void PosOriTask::setFullAngularMotionControl()
{
	_sigma_orientation = Eigen::Matrix3d::Identity();
	_sigma_moment.setZero();

	resetIntegratorsAngular();
}

void PosOriTask::setClosedLoopForceControl()
{
	_closed_loop_force_control = true;
	resetIntegratorsLinear();
}

void PosOriTask::setOpenLoopForceControl()
{
	_closed_loop_force_control = false;
}

void PosOriTask::setClosedLoopMomentControl()
{
	_closed_loop_moment_control = true;
	resetIntegratorsAngular();
}

void PosOriTask::setOpenLoopMomentControl()
{
	_closed_loop_moment_control = false;
}

void PosOriTask::resetIntegrators()
{
	_integrated_orientation_error.setZero();
	_integrated_position_error.setZero();
	_integrated_force_error.setZero();
	_integrated_moment_error.setZero();
	_first_iteration = true;	
}

void PosOriTask::resetIntegratorsLinear()
{
	_integrated_position_error.setZero();
	_integrated_force_error.setZero();
}

void PosOriTask::resetIntegratorsAngular()
{
	_integrated_orientation_error.setZero();
	_integrated_moment_error.setZero();
}


} /* namespace Sai2Primitives */

