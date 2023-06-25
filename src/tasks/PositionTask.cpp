/*
 * PositionTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "PositionTask.h"

#include <stdexcept>

using namespace std;
using namespace Eigen;

namespace Sai2Primitives
{


PositionTask::PositionTask(Sai2Model::Sai2Model* robot,
			const string link_name, 
			const Affine3d control_frame,
			const double loop_time) :
	PositionTask(robot, link_name, control_frame.translation(), control_frame.linear(), loop_time) {}

PositionTask::PositionTask(Sai2Model::Sai2Model* robot, 
			const string link_name, 
			const Vector3d pos_in_link, 
			const Matrix3d rot_in_link,
			const double loop_time)
{

	Affine3d control_frame = Affine3d::Identity();
	control_frame.linear() = rot_in_link;
	control_frame.translation() = pos_in_link;

	_robot = robot;
	_link_name = link_name;
	_control_frame = control_frame;

	int dof = _robot->dof();

	_T_control_to_sensor = Affine3d::Identity();

	_robot->position(_current_position, _link_name, _control_frame.translation());
	_current_velocity.setZero();

	// default values for gains and velocity saturation
	_use_velocity_saturation_flag = false;
	_saturation_velocity = 0.3;
	_kp = 50.0;
	_kv = 14.0;
	_ki = 0.0;

	_use_isotropic_gains = true;
	_kp_mat.setZero();
	_kv_mat.setZero();
	_ki_mat.setZero();

	_kp_force = 0.7;
	_ki_force = 1.3;
	_kv_force = 10.0;

	_k_ff = 1.0;

	// initialize matrices sizes
	_jacobian.setZero(3,dof);
	_projected_jacobian.setZero(3,dof);
	_Lambda.setZero(3,3);
	_Lambda_modified.setZero(3,3);
	_Jbar.setZero(dof,3);
	_N.setZero(dof,dof);
	_N_prec = MatrixXd::Identity(dof,dof);

	_URange = MatrixXd::Identity(3,3);
	_unconstrained_dof = 3;

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
	int dof = _robot->dof();

	_robot->position(_current_position, _link_name, _control_frame.translation());
	_robot->position(_desired_position, _link_name, _control_frame.translation());
	_desired_velocity.setZero();
	_desired_acceleration.setZero();
	_desired_force.setZero();

	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;
	_step_desired_acceleration = _desired_acceleration;

	_integrated_position_error.setZero();

	_sensed_force.setZero();
	_integrated_force_error.setZero();

	_closed_loop_force_control = false;

	_passivity_enabled = true;
	_passivity_observer = 0;
	_E_correction = 0;
	_stored_energy_PO = 0;
	_PO_buffer_window = queue<double>();
	_PO_counter = _PO_max_counter;
	_vc_squared_sum = 0;
	_vc.setZero();
	_Rc = 1.0;

	_motion_control.setZero();
	_force_control.setZero();
	_task_force.setZero();
	_unit_mass_force.setZero();
	_first_iteration = true;

#ifdef USING_OTG 
	_otg->reInitialize(_current_position);
#endif
}


void PositionTask::updateTaskModel(const MatrixXd N_prec)
{
	if(N_prec.rows() != N_prec.cols())
	{
		throw invalid_argument("N_prec matrix not square in PositionTask::updateTaskModel\n");
	}
	if(N_prec.rows() != _robot->dof())
	{
		throw invalid_argument("N_prec matrix size not consistent with robot dof in PositionTask::updateTaskModel\n");
	}

	_N_prec = N_prec;

	_robot->Jv(_jacobian, _link_name, _control_frame.translation());
	_projected_jacobian = _jacobian * _N_prec;

	_robot->URangeJacobian(_URange, _projected_jacobian, _N_prec);
	_unconstrained_dof = _URange.cols();

	_robot->operationalSpaceMatrices(_Lambda, _Jbar, _N, _URange.transpose() * _projected_jacobian, _N_prec);

	switch(_dynamic_decoupling_type)
	{
		case FULL_DYNAMIC_DECOUPLING :
		{
			_Lambda_modified = _Lambda;
			break;
		}

		case IMPEDANCE :
		{
			_Lambda_modified = MatrixXd::Identity(_unconstrained_dof,_unconstrained_dof);
			break;
		}

		default :
		{
			_Lambda_modified = _Lambda;
			break;			
		}
	}
}


void PositionTask::computeTorques(VectorXd& task_joint_torques)
{
	_robot->Jv(_jacobian, _link_name, _control_frame.translation());
	_projected_jacobian = _jacobian * _N_prec;

	// get time since last call for the I term
	_t_curr = chrono::high_resolution_clock::now();
	if(_first_iteration)
	{
		_t_prev = _t_curr;
		_first_iteration = false;
	}
	_t_diff = _t_curr - _t_prev;

	// update matrix gains
	if(_use_isotropic_gains)
	{
		_kp_mat = _kp * Matrix3d::Identity();
		_kv_mat = _kv * Matrix3d::Identity();
		_ki_mat = _ki * Matrix3d::Identity();
	}

	Vector3d force_feedback_related_force = Vector3d::Zero();
	Vector3d position_related_force = Vector3d::Zero();

	// update constroller state
	_robot->position(_current_position, _link_name, _control_frame.translation());
	_current_velocity = _projected_jacobian * _robot->dq();

	_step_desired_position = _desired_position;
	_step_desired_velocity = _desired_velocity;
	_step_desired_acceleration = _desired_acceleration;

	// force control
	if(_closed_loop_force_control)
	{
		// update the integrated error
		_integrated_force_error += (_sensed_force - _desired_force) * _t_diff.count();

		// compute the feedback term
		Vector3d force_feedback_term = - _kp_force * (_sensed_force - _desired_force) - _ki_force * _integrated_force_error;
		_vc = force_feedback_term;

		// saturate the feedback term
		if(_vc.norm() > 20.0)
		{
			_vc *= 20.0 / _vc.norm();
		}

		// implement passivity observer and controller
		if(_passivity_enabled)
		{
			Vector3d vc_force_space = _sigma_force * _vc;
			Vector3d vr_force_space = _sigma_force * _current_velocity;
			Vector3d F_cmd = _k_ff * _desired_force + _Rc * _vc - _kv_force * vr_force_space;
			double vc_squared = _vc.transpose() * _sigma_force * _vc;
			Vector3d f_diff = _sensed_force - _desired_force;

			// compute power input and output
			double power_input_output = (f_diff.dot(vc_force_space) - F_cmd.dot(vr_force_space)) * _t_diff.count();

			// compute stored energy (intentionally diabled)
			// _stored_energy_PO = 0.5 * _ki_force * (double) (_integrated_force_error.transpose() * _sigma_force * _integrated_force_error);

			// windowed PO
			_passivity_observer += power_input_output;
			_PO_buffer_window.push(power_input_output);

			if(_passivity_observer + _stored_energy_PO + _E_correction > 0)
			{
				while(_PO_buffer_window.size() > _PO_window_size)
				{
					if(_passivity_observer + _E_correction + _stored_energy_PO > _PO_buffer_window.front())
					{
						if(_PO_buffer_window.front() > 0)
						{
							_passivity_observer -= _PO_buffer_window.front();
						}
						_PO_buffer_window.pop();
					}
					else
					{
						break;
					}
				}
			}

			// compute PC
			if(_PO_counter <= 0)
			{
				_PO_counter = _PO_max_counter;

				double old_Rc = _Rc;
				if(_passivity_observer + _stored_energy_PO + _E_correction < 0)   // activity detected
				{
					_Rc = 1 + (_passivity_observer + _stored_energy_PO + _E_correction) / (_vc_squared_sum * _t_diff.count());

					if(_Rc > 1){_Rc = 1;}
					if(_Rc < 0){_Rc = 0;}

				}
				else    // no activity detected
				{
					_Rc = (1 + (0.1*_PO_max_counter-1)*_Rc)/(double)(0.1*_PO_max_counter);
				}

				_E_correction += (1 - old_Rc) * _vc_squared_sum * _t_diff.count();
				_vc_squared_sum = 0;
			}

			_PO_counter--;
			_vc_squared_sum += vc_squared;
		}
		else  // no passivity enabled
		{
			_Rc = 1;
		}

		// compute the final contribution
		force_feedback_related_force = _sigma_force * ( _Rc * _vc - _kv_force * _current_velocity);
	}
	else // open loop force control
	{
		force_feedback_related_force = _sigma_force * (- _kv_force * _current_velocity);
	}

	// motion related terms
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
		_step_desired_velocity = -_kp_mat * _kv_mat.inverse() * (_current_position - _step_desired_position) - _ki_mat * _kv_mat.inverse() * _integrated_position_error;
		if(_step_desired_velocity.norm() > _saturation_velocity)
		{
			_step_desired_velocity *= _saturation_velocity / _step_desired_velocity.norm();
		}
		_motion_control = _sigma_motion * (_step_desired_acceleration -_kv_mat*(_current_velocity - _step_desired_velocity));
	}
	else
	{
		_motion_control = _sigma_motion * (_step_desired_acceleration -_kp_mat*(_current_position - _step_desired_position) - _kv_mat*(_current_velocity - _step_desired_velocity ) - _ki_mat * _integrated_position_error);
	}

	_force_control = _sigma_force * _k_ff * _desired_force + force_feedback_related_force;

	// compute task torques
	_task_force = _Lambda_modified * _URange.transpose() * _motion_control + _URange.transpose() * _force_control;
	task_joint_torques = _projected_jacobian.transpose() * _URange * _task_force;

	// update previous time
	_t_prev = _t_curr;
}




bool PositionTask::goalPositionReached(const double tolerance, const bool verbose)
{
	double position_error = (_desired_position - _current_position).transpose() * ( _URange * _sigma_motion * _URange.transpose()) * (_desired_position - _current_position);
	position_error = sqrt(position_error);
	bool goal_reached = position_error < tolerance;
	if(verbose)
	{
		cout << "position error in PositionTask : " << position_error << endl;
		cout << "Tolerance : " << tolerance << endl;
		cout << "Goal reached : " << goal_reached << endl << endl;
	}

	return goal_reached;
}

void PositionTask::setDynamicDecouplingFull()
{
	_dynamic_decoupling_type = FULL_DYNAMIC_DECOUPLING;
}

void PositionTask::setDynamicDecouplingNone()
{
	_dynamic_decoupling_type = IMPEDANCE;
}

void PositionTask::setNonIsotropicGains(const Matrix3d& frame, const Vector3d& kp, const Vector3d& kv, const Vector3d& ki)
{
	if( (Matrix3d::Identity() - frame.transpose()*frame).norm() > 1e-3 || frame.determinant() < 0)
	{
		throw invalid_argument("not a valid right hand frame in PositionTask::setNonIsotropicGainsPosition\n");
	}

	_use_isotropic_gains = false;

	Matrix3d kp_mat_tmp = Matrix3d::Zero();
	Matrix3d kv_mat_tmp = Matrix3d::Zero();
	Matrix3d ki_mat_tmp = Matrix3d::Zero();

	for(int i=0 ; i<3 ; i++)
	{
		kp_mat_tmp(i,i) = kp(i);
		kv_mat_tmp(i,i) = kv(i);
		ki_mat_tmp(i,i) = ki(i);
	}

	_kp_mat = frame * kp_mat_tmp * frame.transpose();
	_kv_mat = frame * kv_mat_tmp * frame.transpose();
	_ki_mat = frame * ki_mat_tmp * frame.transpose();
}

void PositionTask::setIsotropicGains(const double kp, const double kv, const double ki)
{
	_use_isotropic_gains = true;

	_kp = kp;
	_kv = kv;
	_ki = ki;
}

void PositionTask::setForceSensorFrame(const std::string link_name, const Affine3d transformation_in_link)
{
	if(link_name != _link_name)
	{
		throw invalid_argument("The link to which is attached the sensor should be the same as the link to which is attached the control frame in PositionTask::setForceSensorFrame\n");
	}
	_T_control_to_sensor = _control_frame.inverse() * transformation_in_link;
}

void PositionTask::updateSensedForceAndMoment(const Vector3d sensed_force_sensor_frame)
{
	// find the transform from base frame to control frame
	Affine3d T_base_link;
	_robot->transform(T_base_link, _link_name);
	Affine3d T_base_control = T_base_link * _control_frame;

	// find the resolved sensed force in control frame
	_sensed_force = _T_control_to_sensor.rotation() * sensed_force_sensor_frame;

	// rotate the quantities in base frame
	_sensed_force = T_base_control.rotation() * _sensed_force;
}

void PositionTask::setForceAxis(const Vector3d force_axis)
{
	Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis*normalized_axis.transpose();
	_sigma_motion = Matrix3d::Identity() - _sigma_force;

	resetIntegrators();
#ifdef USING_OTG 
	_otg->reInitialize(_current_position);
#endif
}


void PositionTask::updateForceAxis(const Vector3d force_axis)
{
	Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis*normalized_axis.transpose();
	_sigma_motion = Matrix3d::Identity() - _sigma_force;
}


void PositionTask::setLinearMotionAxis(const Vector3d motion_axis)
{
	Vector3d normalized_axis = motion_axis.normalized();

	_sigma_motion = normalized_axis*normalized_axis.transpose();
	_sigma_force = Matrix3d::Identity() - _sigma_motion;

	resetIntegrators();
#ifdef USING_OTG 
	_otg->reInitialize(_current_position);
#endif
}


void PositionTask::updateLinearMotionAxis(const Vector3d motion_axis)
{
	Vector3d normalized_axis = motion_axis.normalized();

	_sigma_motion = normalized_axis*normalized_axis.transpose();
	_sigma_force = Matrix3d::Identity() - _sigma_motion;	
}

void PositionTask::setFullForceControl()
{
	_sigma_force = Matrix3d::Identity();
	_sigma_motion.setZero();

	resetIntegrators();
#ifdef USING_OTG 
	_otg->reInitialize(_current_position);
#endif
}

void PositionTask::setFullLinearMotionControl()
{
	_sigma_motion = Matrix3d::Identity();
	_sigma_force.setZero();

	resetIntegrators();
#ifdef USING_OTG 
	_otg->reInitialize(_current_position);
#endif
}

void PositionTask::setClosedLoopForceControl()
{
	_closed_loop_force_control = true;
	resetIntegrators();
}

void PositionTask::setOpenLoopForceControl()
{
	_closed_loop_force_control = false;
}

void PositionTask::enablePassivity()
{
	_passivity_enabled = true;
}

void PositionTask::disablePassivity()
{
	_passivity_enabled = false;
}

void PositionTask::resetIntegrators()
{
	_integrated_position_error.setZero();
	_integrated_force_error.setZero();
	_first_iteration = true;
}

} /* namespace Sai2Primitives */

