/*
 * OpenLoopTeleop.cpp
 *
 *      Author: Margot
 */

#include "OpenLoopTeleop.h"

#include <stdexcept>


namespace Sai2Primitives
{


OpenLoopTeleop::~OpenLoopTeleop ()
{
	_force_dev.setZero();
	_torque_dev.setZero();
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);	
	hapticDevice->close();
	delete handler;
	handler = NULL;

	_pos_rob = _centerPos_rob;
	_rot_rob = _centerRot_rob;

	delete _force_filter;
	_force_filter = NULL;
	delete _moment_filter;
	_moment_filter = NULL;
}

OpenLoopTeleop::OpenLoopTeleop()
{
	// create a haptic device handler
    auto handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);
	if (NULL == hapticDevice) {
		cout << "No haptic device found. " << endl;
		device_started = false;
	} else {
		hapticDevice->open();
		hapticDevice->calibrate();
		device_started = true;
	}

	//Send zero force feedback to the haptic device
	_force_dev.setZero();
	_torque_dev.setZero();
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);

	// retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
	// get properties of haptic device 
	maxForce_dev = hapticDeviceInfo.m_maxLinearForce;
	maxTorque_dev = hapticDeviceInfo.m_maxAngularTorque;
	maxLinDamping_dev = hapticDeviceInfo.m_maxLinearDamping;
	maxAngDamping_dev = hapticDeviceInfo.m_maxAngularDamping;
	maxLinStiffness_dev = hapticDeviceInfo.m_maxLinearStiffness;
	maxAngStiffness_dev = hapticDeviceInfo.m_maxAngularStiffness;


	//Initialize homing task
	device_homed = false;
	// Default home position (can be upload with setDeviceCenter())
	_HomePos_op.setZero();
	_HomeRot_op.setIdentity();

	// Initialize workspace center of the controlled robot (can be upload with setRobotCenter())
	_centerPos_rob.setZero(); 
	_centerRot_rob.setIdentity();

	//Initialize the frame trasform from device to robot
	_transformDev_Rob.setIdentity();

	//Initialize the set position and orientation of the controlled robot
	_pos_rob = _centerPos_rob;
	_rot_rob = _centerRot_rob;

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

	// To initialize the timer
	_first_iteration = true;

}

void OpenLoopTeleop::GavityCompTask()
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

	if( (_pos_dev - _HomePos_op).norm()<0.002 && orientation_error.norm() < 0.04)
	{
		device_homed = true;
	}
}


void OpenLoopTeleop::computeHapticCommands_Impedance(
				Eigen::Vector3d& pos_rob,
				Eigen::Matrix3d& rot_rob,
				const Eigen::VectorXd f_task_sensed,
				const Eigen::Vector3d pos_rob_sensed, 
		        const Eigen::Matrix3d rot_rob_sensed)
{
	// get time since last call
	_t_curr = std::chrono::high_resolution_clock::now();
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
		_pos_rob = _centerPos_rob;
		_rot_rob = _centerRot_rob;
	}
	_t_diff = _t_curr - _t_prev;


	device_homed = false;
	// read haptic device position
	hapticDevice->getPosition(_pos_dev_chai);
	hapticDevice->getRotation(_rot_dev_chai);
	_pos_dev = convertChaiToEigenVector(_pos_dev_chai);
	_rot_dev = convertChaiToEigenMatrix(_rot_dev_chai);

	// Compute the force feedback in robot frame
	Vector3d f_task_trans;
	Vector3d f_task_rot;
	Vector3d orientation_dev;
		
	if (f_task_sensed.norm() <= 0.0001)
	{
		f_task_trans = _Red_factor_trans * f_task_sensed.head(3);
		f_task_rot = _Red_factor_rot * f_task_sensed.tail(3);
	}
	else
	{
		//Evaluate the task force through stiffness proxy
		f_task_trans = _k_pos*(pos_rob_sensed - _pos_rob);
		// Compute the orientation error
		Sai2Model::orientationError(orientation_dev, _rot_rob, rot_rob_sensed);
		// Evaluate task torque
		f_task_rot = _k_ori*orientation_dev;
		// Apply reduction factors to force feedback
		f_task_trans = _Red_factor_trans * f_task_trans;
		f_task_rot = _Red_factor_rot * f_task_rot;
	}

	//Transfer task force from robot to haptic device global frame
	_force_dev = _transformDev_Rob * f_task_trans;
	_torque_dev = _transformDev_Rob * f_task_rot;

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
	Eigen::Matrix3d rot_rel = _HomeRot_op.transpose()*_rot_dev;
	Eigen::AngleAxisd rot_rel_ang = Eigen::AngleAxisd(rot_rel);
	// Compute set orientation from the haptic device
	Eigen::AngleAxisd rot_rob_aa = Eigen::AngleAxisd(_KsR*rot_rel_ang.angle(),rot_rel_ang.axis());
	_rot_rob = rot_rob_aa.toRotationMatrix();

	//Transfer set position and orientation from device to robot global frame
	_pos_rob = _transformDev_Rob.transpose() * _pos_rob;
	_rot_rob = _transformDev_Rob.transpose() * _rot_rob;

	// Adjust set position and orientation to the center of the task workspace
	_pos_rob = _pos_rob + _centerPos_rob;
	_rot_rob = _rot_rob * _centerRot_rob; //////////////////////////////////////////////////// check matrix product here..

	// Send set position orientation of the robot
	pos_rob = _pos_rob;
	rot_rob = _rot_rob;

	// update previous time
	_t_prev = _t_curr;
}

void OpenLoopTeleop::computeHapticCommands_ForceSensor(
				Eigen::Vector3d& pos_rob,
				Eigen::Matrix3d& rot_rob,
				const Eigen::VectorXd f_task_sensed,
				const bool filter_on)
{
	// get time since last call
	_t_curr = std::chrono::high_resolution_clock::now();
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
	}
	_t_diff = _t_curr - _t_prev;


	device_homed = false;
	// read haptic device position
	hapticDevice->getPosition(_pos_dev_chai);
	hapticDevice->getRotation(_rot_dev_chai);
	_pos_dev = convertChaiToEigenVector(_pos_dev_chai);
	_rot_dev = convertChaiToEigenMatrix(_rot_dev_chai);


	// Compute the force feedback in robot frame
	// Filtering sensed forced
	Vector3d f_task_trans_sensed = f_task_sensed.head(3);
	Vector3d f_task_rot_sensed = f_task_sensed.tail(3);

	if (filter_on == true)
	{
		f_task_trans_sensed = _force_filter->update(f_task_trans_sensed);
		f_task_rot_sensed = _moment_filter->update(f_task_rot_sensed);
	}

	Vector3d f_task_trans;
	Vector3d f_task_rot;	
	f_task_trans = _Red_factor_trans * f_task_trans_sensed;
	f_task_rot = _Red_factor_rot * f_task_rot_sensed;

	//Transfer task force from robot to haptic device global frame
	_force_dev = _transformDev_Rob * f_task_trans;
	_torque_dev = _transformDev_Rob * f_task_rot;

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
	Eigen::Matrix3d rot_rel = _HomeRot_op.transpose()*_rot_dev;
	Eigen::AngleAxisd rot_rel_ang = Eigen::AngleAxisd(rot_rel);
	// Compute set orientation from the haptic device
	Eigen::AngleAxisd rot_rob_aa = Eigen::AngleAxisd(_KsR*rot_rel_ang.angle(),rot_rel_ang.axis());
	_rot_rob = rot_rob_aa.toRotationMatrix();

	//Transfer set position and orientation from device to robot global frame
	_pos_rob = _transformDev_Rob.transpose() * _pos_rob;
	_rot_rob = _transformDev_Rob.transpose() * _rot_rob;

	// Adjust set position and orientation to the center of the task workspace
	_pos_rob = _pos_rob + _centerPos_rob;
	_rot_rob = _rot_rob * _centerRot_rob; //////////////////////////////////////////////////// check matrix product here..

	// Send set position orientation of the robot
	pos_rob = _pos_rob;
	rot_rob = _rot_rob;

	// update previous time
	_t_prev = _t_curr;
}



void OpenLoopTeleop::reInitializeTask()
{

	//Send zero force feedback to the haptic device
	_force_dev.setZero();
	_torque_dev.setZero();
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);

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

	_first_iteration = true; // To initialize the timer
}


void OpenLoopTeleop::setScalingFactors(const double Ks, const double KsR)
{
	_Ks = Ks;
	_KsR = KsR;
}

void setPosCtrlGains (const double kp_pos, const double kv_pos, const double kp_ori, const double kv_ori)
{
	_kp_pos = kp_pos;
	_kv_pos = kv_pos;
	_kp_ori = kp_ori;
	_kv_ori = kv_ori;
}

void setForceFeedbackCtrlGains (const double k_pos, const double k_ori,
		const Matrix3d Red_factor_rot,
		const Matrix3d Red_factor_trans)
{
	_k_pos = k_pos;
	_k_ori = k_ori;
	_Red_factor_rot = Red_factor_rot;
	_Red_factor_trans = Red_factor_trans;
}

void setFilterCutOffFreq(const double fc_force, const double fc_moment)
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

void OpenLoopTeleop::setDeviceRobotTransform(const Eigen::Matrix3d transformDev_Rob)
{
	_transformDev_Rob = transformDev_Rob;
}

void OpenLoopTeleop::setRobotCenter(const Eigen::Vector3d centerPos_rob, const Eigen::Matrix3d centerRot_rob)
{
	_centerPos_rob = centerPos_rob; 
	_centerRot_rob = centerRot_rob;

	//Initialize the set position and orientation of the controlled robot
	_pos_rob = _centerPos_rob;
	_rot_rob = _centerRot_rob;

}

void OpenLoopTeleop::EnableGripperUserSwitch()
{
	// if the device has a gripper, then enable it to behave like a user switch
	hapticDevice->setEnableGripperUserSwitch(true);
	gripper_state = false;
}

void OpenLoopTeleop::ReadGripperUserSwitch()
{
	// Read new gripper state
    hapticDevice->getUserSwitch (0, gripper_state);
}

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



} /* namespace Sai2Primitives */

