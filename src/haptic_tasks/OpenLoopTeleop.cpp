/*
 * OpenLoopTeleop.cpp
 *
 *      This controller implements the bilateral teleoperation scheme in open loop.
 *	It computes the haptic device force feedback and the controlled robot set position
 *	with respect to the input command of the device. 
 *
 *      Author: Margot Vulliez & Mikael Jorda
 *
 */

#include "OpenLoopTeleop.h"

#include <stdexcept>


namespace Sai2Primitives
{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Declaration of internal functions ////
cVector3d convertEigenToChaiVector( Eigen::Vector3d a_vec )
{
    double x = a_vec(0);
    double y = a_vec(1);
    double z = a_vec(2);
    return cVector3d(x,y,z);
}

Eigen::Vector3d convertChaiToEigenVector( cVector3d a_vec )
{
    double x = a_vec.x();
    double y = a_vec.y();
    double z = a_vec.z();
    return Eigen::Vector3d(x,y,z);
}

cMatrix3d convertEigenToChaiRotation( Eigen::Matrix3d a_mat )
{
    return cMatrix3d( a_mat(0,0), a_mat(0,1), a_mat(0,2), a_mat(1,0), a_mat(1,1), a_mat(1,2), a_mat(2,0), a_mat(2,1), a_mat(2,2) );
}

Eigen::Matrix3d convertChaiToEigenMatrix( cMatrix3d a_mat )
{
    Eigen::Matrix3d asdf;
    asdf << a_mat(0,0), a_mat(0,1), a_mat(0,2),
            a_mat(1,0), a_mat(1,1), a_mat(1,2),
            a_mat(2,0), a_mat(2,1), a_mat(2,2);

    return asdf;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Constructor / Destructor ////

OpenLoopTeleop::OpenLoopTeleop(cHapticDeviceHandler* handler,
								const int device_index,
								const Eigen::Vector3d centerPos_rob, 
		            			const Eigen::Matrix3d centerRot_rob,
		            			const Eigen::Matrix3d transformDev_Rob)
{
	// open and calibrate the haptic device
	handler->getDevice(hapticDevice, device_index);	
		if (NULL == hapticDevice)
		{
			cout << "No haptic device found. " << endl;
			device_started = false;
		}
		else
		{
			if(!hapticDevice->open())
			{
				cout << "could not open the haptic device" << endl;
			}
			if(!hapticDevice->calibrate())
			{
				cout << "could not calibrate the haptic device" << endl;
			}
			else
			{
				device_started = true;
			}
		}

	// read info from haptic device
	cHapticDeviceInfo hapticDeviceInfo;
	handler->getDeviceSpecifications(hapticDeviceInfo, device_index);

	// get properties of haptic device 
	maxForce_dev = hapticDeviceInfo.m_maxLinearForce;
	maxTorque_dev = hapticDeviceInfo.m_maxAngularTorque;
	maxLinDamping_dev = hapticDeviceInfo.m_maxLinearDamping;
	maxAngDamping_dev = hapticDeviceInfo.m_maxAngularDamping;
	maxLinStiffness_dev = hapticDeviceInfo.m_maxLinearStiffness;
	maxAngStiffness_dev = hapticDeviceInfo.m_maxAngularStiffness;

	//Send zero force feedback to the haptic device
	_force_dev.setZero();
	_torque_dev.setZero();
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);

	//Initialize snesed force to zero
	_f_task_sensed.setZero(6);

	//Initialize homing task
	device_homed = false;
	// Default home position (can be upload with setDeviceCenter())
	_HomePos_op.setZero();
	_HomeRot_op.setIdentity();

	// Initialize workspace center of the controlled robot (can be upload with setRobotCenter())
	_centerPos_rob = centerPos_rob; 
	_centerRot_rob = centerRot_rob;

	//Initialize the frame trasform from device to robot
	_transformDev_Rob = transformDev_Rob;

	//Initialize the set position and orientation of the controlled robot
	_pos_rob = _centerPos_rob;
	_rot_rob = _centerRot_rob;
	_pos_rob_sensed = _centerPos_rob;
	_rot_rob_sensed = _centerRot_rob;

	// Initialize scaling factors (can be set through setScalingFactors())
	_Ks=1.0;
	_KsR=1.0;

	//Initialize position controller parameters
	_kp_pos = 0.1;
	_kv_pos = 0.5;
	_kp_ori = 0.4;
	_kv_ori = 0.2;

	//Initialize force feedback controller parameters
	_k_pos = 200.0;
	_k_ori = 15.0;
	_d_pos = 10.0;
	_d_ori = 5.0;
	_Red_factor_rot << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	_Red_factor_trans << 1/2.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0,
						  0.0, 0.0, 1/2.0;

	//Initialize sensed force filtering
	_force_filter = new ButterworthFilter(3);
	_moment_filter = new ButterworthFilter(3);

	_fc_force = 0.02;
	_fc_moment = 0.02;
	_force_filter->setCutoffFrequency(_fc_force);
	_moment_filter->setCutoffFrequency(_fc_moment);
	filter_on = false;

	//Initialiaze force feedback computation mode
	proxy = false;
	position_only = false;

	// To initialize the timer
	_first_iteration = true;

}


OpenLoopTeleop::~OpenLoopTeleop ()
{
	_force_dev.setZero();
	_torque_dev.setZero();
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);	
	hapticDevice->close();

	_pos_rob = _centerPos_rob;
	_rot_rob = _centerRot_rob;

	delete _force_filter;
	_force_filter = NULL;
	delete _moment_filter;
	_moment_filter = NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Core methods ////

void OpenLoopTeleop::computeHapticCommands(Eigen::Vector3d& pos_rob,
							Eigen::Matrix3d& rot_rob)
{
	// get time since last call
	// _t_curr = std::chrono::high_resolution_clock::now();
	// if(_first_iteration)
	// {
	// 	_t_prev = std::chrono::high_resolution_clock::now();
	// 	_first_iteration = false;
	// 	_pos_rob = _centerPos_rob;
	// 	_rot_rob = _centerRot_rob;
	// }
	// _t_diff = _t_curr - _t_prev;

	device_homed = false;
	// read haptic device position
	hapticDevice->getPosition(_pos_dev_chai);
	_pos_dev = convertChaiToEigenVector(_pos_dev_chai);
	if (!position_only)
	{
		hapticDevice->getRotation(_rot_dev_chai);
		_rot_dev = convertChaiToEigenMatrix(_rot_dev_chai);
	}

	// Compute the force feedback in robot frame
	Vector3d f_task_trans;
	Vector3d f_task_rot;
	Vector3d orientation_dev;
	
	if (proxy)
	{
		// if ((pos_rob_sensed - _pos_rob).norm() <= 0.01)
	// {
	// 	f_task_trans.setZero();
	// 	f_task_rot.setZero();
	// }
	// else
	// {
		// Evaluate the task force through stiffness proxy
		f_task_trans = _k_pos*(_pos_rob_sensed - _pos_rob) - _d_pos * _vel_rob_trans;
		if (!position_only)
		{
			// Compute the orientation error
			Sai2Model::orientationError(orientation_dev, _rot_rob, _rot_rob_sensed);
			// Evaluate task torque
			f_task_rot = _k_ori*orientation_dev - _d_ori * _vel_rob_rot;
		}
		else 
		{
			f_task_rot.setZero();
		}
	// }
	}
	else
	{
		// Read sensed task force 
		f_task_trans = _f_task_sensed.head(3);
		if (!position_only)
		{
		f_task_rot = _f_task_sensed.tail(3);
		}
		else
		{
			f_task_rot.setZero();
		}
	}
	
	// Apply reduction factors to force feedback
	f_task_trans = _Red_factor_trans * f_task_trans;
	f_task_rot = _Red_factor_rot * f_task_rot;
	//Transfer task force from robot to haptic device global frame
	_force_dev = _transformDev_Rob * f_task_trans;
	_torque_dev = _transformDev_Rob * f_task_rot;

	// Scaling of the force feedback
	Eigen::Matrix3d scaling_factor_trans;
	Eigen::Matrix3d scaling_factor_rot;

		scaling_factor_trans << 1/_Ks, 0.0, 0.0,
						  0.0, 1/_Ks, 0.0, 
						  0.0, 0.0, 1/_Ks;
		scaling_factor_rot << 1/_KsR, 0.0, 0.0, 
						  0.0, 1/_KsR, 0.0,
						  0.0, 0.0, 1/_KsR;

	_force_dev = scaling_factor_trans * _force_dev;
	_torque_dev = scaling_factor_rot * _torque_dev;

	// Saturate to Force and Torque limits of the haptic device
	if (_force_dev.norm() >= maxForce_dev)
	{
		_force_dev = maxForce_dev*_force_dev/(_force_dev.norm());
	}
	if (_torque_dev.norm() >= maxTorque_dev)
	{
		_torque_dev = maxTorque_dev*_torque_dev/(_torque_dev.norm());
	}

	// Send controllers force and torque to haptic device
	_force_chai = convertEigenToChaiVector(_force_dev);
	_torque_chai = convertEigenToChaiVector(_torque_dev);
    hapticDevice->setForceAndTorque(_force_chai,_torque_chai);


 	// Compute the set position from the haptic device
	_pos_rob = _Ks*(_pos_dev-_HomePos_op);
	// Rotation with respect with home orientation
	if (!position_only)
	{
		Eigen::Matrix3d rot_rel = _rot_dev * _HomeRot_op.transpose();
		Eigen::AngleAxisd rot_rel_ang = Eigen::AngleAxisd(rot_rel);

		// Compute set orientation from the haptic device
		Eigen::AngleAxisd rot_rob_aa = Eigen::AngleAxisd(_KsR*rot_rel_ang.angle(),rot_rel_ang.axis());
		_rot_rob = rot_rob_aa.toRotationMatrix();
	}


	//Transfer set position and orientation from device to robot global frame
	_pos_rob = _transformDev_Rob.transpose() * _pos_rob;
	if (!position_only)
	{
		_rot_rob = _transformDev_Rob.transpose() * _rot_rob * _transformDev_Rob * _centerRot_rob; 
	}
	else 
	{
		_rot_rob.setIdentity();
	}

	// Adjust set position to the center of the task workspace
	_pos_rob = _pos_rob + _centerPos_rob;

	// Send set position orientation of the robot
	pos_rob = _pos_rob;
	rot_rob = _rot_rob;

	// update previous time
	// _t_prev = _t_curr;
}

void OpenLoopTeleop::updateSensedForce(const Eigen::VectorXd f_task_sensed)
{
	if (filter_on)
	{
		Vector3d f_task_trans_sensed = f_task_sensed.head(3);
		Vector3d f_task_rot_sensed = f_task_sensed.tail(3);
		f_task_trans_sensed = _force_filter->update(f_task_trans_sensed);
		f_task_rot_sensed = _moment_filter->update(f_task_rot_sensed);

		_f_task_sensed << f_task_trans_sensed, f_task_rot_sensed;
	}
	else
	{
		_f_task_sensed = f_task_sensed;
	}
}

void OpenLoopTeleop::updateSensedRobotPositionVelocity(const Eigen::Vector3d pos_rob_sensed,
								const Eigen::Vector3d vel_rob_trans,
								const Eigen::Matrix3d rot_rob_sensed,
								const Eigen::Vector3d vel_rob_rot)
{
	_pos_rob_sensed = pos_rob_sensed;
	_rot_rob_sensed = rot_rob_sensed;
	_vel_rob_trans = vel_rob_trans;
	_vel_rob_rot = vel_rob_rot;

}


void OpenLoopTeleop::GravityCompTask()
{
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);
}

void OpenLoopTeleop::HomingTask()
{	
	// Haptice device position controller gains
	double kp_pos =_kp_pos * maxLinStiffness_dev;
	double kv_pos =_kv_pos * maxLinDamping_dev;
	double kp_ori =_kp_ori * maxAngStiffness_dev;
	double kv_ori =_kv_ori * maxAngDamping_dev;

	// read haptic device position and velocity
	hapticDevice->getPosition(_pos_dev_chai);
	hapticDevice->getRotation(_rot_dev_chai);
	_pos_dev = convertChaiToEigenVector(_pos_dev_chai);
	_rot_dev = convertChaiToEigenMatrix(_rot_dev_chai);
	hapticDevice->getLinearVelocity(_vel_dev_trans_chai);
	hapticDevice->getAngularVelocity(_vel_dev_rot_chai);
	_vel_dev_trans = convertChaiToEigenVector(_vel_dev_trans_chai);
	_vel_dev_rot = convertChaiToEigenVector(_vel_dev_rot_chai);

	// Evaluate position controller force
	_force_dev = -kp_pos*(_pos_dev - _HomePos_op) - kv_pos * _vel_dev_trans;
	// Compute the orientation error
	Vector3d orientation_error;
	Sai2Model::orientationError(orientation_error, _HomeRot_op, _rot_dev);
	// Evaluate orientation controller force
	_torque_dev = -kp_ori*orientation_error - kv_ori * _vel_dev_rot;

	// Saturate to Force and Torque limits of the haptic device
	if (_force_dev.norm() >= maxForce_dev)
	{
		_force_dev = maxForce_dev*_force_dev/(_force_dev.norm());
	}
	if (_torque_dev.norm() >= maxTorque_dev)
	{
		_torque_dev = maxTorque_dev*_torque_dev/(_torque_dev.norm());
	}

	// Send controllers force and torque to haptic device
	_force_chai = convertEigenToChaiVector(_force_dev);
	_torque_chai = convertEigenToChaiVector(_torque_dev);
    hapticDevice->setForceAndTorque(_force_chai,_torque_chai);
    //&& orientation_error.norm() < 0.04
	if( (_pos_dev - _HomePos_op).norm()<0.002)
	{
		device_homed = true;
	}
}

void OpenLoopTeleop::EnableGripperUserSwitch()
{
	// if the device has a gripper, then enable it to behave like a user switch
	hapticDevice->setEnableGripperUserSwitch(true);
	gripper_state = false;
}

bool OpenLoopTeleop::ReadGripperUserSwitch()
{
	// Read new gripper state
    hapticDevice->getUserSwitch (0, gripper_state);
    return gripper_state;
}

void OpenLoopTeleop::reInitializeTask()
{

	//Send zero force feedback to the haptic device
	_force_dev.setZero();
	_torque_dev.setZero();
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);

	//reInitialize snesed force to zero
	_f_task_sensed.setZero(6);

	//reInitialize homing task
	device_homed = false;
	// Default home position
	_HomePos_op.setZero();
	_HomeRot_op.setIdentity();

	//reInitialize workspace center of the controlled robot to defaut values
	_centerPos_rob.setZero(); 
	_centerRot_rob.setIdentity();

	//Initialize the set position and orientation of the controlled robot
	_pos_rob = _centerPos_rob;
	_rot_rob = _centerRot_rob;
	_pos_rob_sensed = _centerPos_rob;
	_rot_rob_sensed = _centerRot_rob;

	//Initialize the frame trasform from device to robot
	_transformDev_Rob.setIdentity();

	//reInitialize scaling factors to defaut values
	_Ks=1.0;
	_KsR=1.0;

	//Initialize position controller parameters
	_kp_pos = 0.1;
	_kv_pos = 0.5;
	_kp_ori = 0.4;
	_kv_ori = 0.2;

	//Initialize force feedback controller parameters
	_k_pos = 200.0;
	_k_ori = 15.0;
	_d_pos = 10.0;
	_d_ori = 5.0;
	_Red_factor_rot << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	_Red_factor_trans << 1/2.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0,
						  0.0, 0.0, 1/2.0;

	//reInitialize filter parameters
	_fc_force = 0.02;
	_fc_moment = 0.02;
	_force_filter->setCutoffFrequency(_fc_force);
	_moment_filter->setCutoffFrequency(_fc_moment);
	filter_on = false;

	//Initialiaze force feedback computation mode
	proxy = false;
	position_only = false;

	_first_iteration = true; // To initialize the timer
}

	// -------- Parameter setting methods --------

void OpenLoopTeleop::setScalingFactors(const double Ks, const double KsR)
{
	_Ks = Ks;
	_KsR = KsR;
}

void OpenLoopTeleop::setPosCtrlGains (const double kp_pos, const double kv_pos, const double kp_ori, const double kv_ori)
{
	_kp_pos = kp_pos;
	_kv_pos = kv_pos;
	_kp_ori = kp_ori;
	_kv_ori = kv_ori;
}

void OpenLoopTeleop::setForceFeedbackCtrlGains (const double k_pos, const double d_pos, const double k_ori, const double d_ori,
		const Matrix3d Red_factor_rot,
		const Matrix3d Red_factor_trans)
{
	_k_pos = k_pos;
	_k_ori = k_ori;
	_d_pos = d_pos;
	_d_ori = d_ori;
	_Red_factor_rot = Red_factor_rot;
	_Red_factor_trans = Red_factor_trans;
}

void OpenLoopTeleop::setFilterCutOffFreq(const double fc_force, const double fc_moment)
{
	_fc_force = fc_force;
	_fc_moment = fc_moment;
	_force_filter->setCutoffFrequency(_fc_force);
	_moment_filter->setCutoffFrequency(_fc_moment);

}

void OpenLoopTeleop::setDeviceCenter(const Eigen::Vector3d HomePos_op, const Eigen::Matrix3d HomeRot_op)
{
	_HomePos_op = HomePos_op;
	_HomeRot_op = HomeRot_op;
}


void OpenLoopTeleop::setRobotCenter(const Eigen::Vector3d centerPos_rob, const Eigen::Matrix3d centerRot_rob)
{
	_centerPos_rob = centerPos_rob; 
	_centerRot_rob = centerRot_rob;

	//Initialize the set position and orientation of the controlled robot
	_pos_rob = _centerPos_rob;
	_rot_rob = _centerRot_rob;

}

void OpenLoopTeleop::setDeviceRobotTransform(const Eigen::Matrix3d transformDev_Rob)
{
	_transformDev_Rob = transformDev_Rob;
}






} /* namespace Sai2Primitives */

