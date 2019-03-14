/*
 * OpenLoopTeleop.cpp
 *
 *      This controller implements the bilateral teleoperation scheme in open loop.
 *	It computes the haptic device force feedback and the controlled robot set position
 *	with respect to the input command of the device. 
 *
 *      Authors: Margot Vulliez & Mikael Jorda
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
								const Eigen::Vector3d center_position_robot, 
		            			const Eigen::Matrix3d center_rotation_robot,
		            			const Eigen::Matrix3d Rotation_Matrix_DeviceToRobot)
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
	handler->getDeviceSpecifications(device_info, device_index);

	//Send zero force feedback to the haptic device
	_commanded_force_device.setZero();
	_commanded_torque_device.setZero();
	cVector3d _force_chai;
	cVector3d _torque_chai;
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);

	//Initialize snesed force to zero
	_sensed_task_force.setZero(6);

	//Initialize homing task
	device_homed = false;
	// Default home position (can be upload with setDeviceCenter())
	_home_position_device.setZero();
	_home_rotation_device.setIdentity();

	// Initialize Workspace center of the controlled robot (can be upload with setRobotCenter())
	_center_position_robot = center_position_robot; 
	_center_rotation_robot = center_rotation_robot;

	//Initialize the frame trasform from device to robot
	_Rotation_Matrix_DeviceToRobot = Rotation_Matrix_DeviceToRobot;

	//Initialize the set position and orientation of the controlled robot
	_desired_position_robot = _center_position_robot;
	_desired_rotation_robot = _center_rotation_robot;
	_current_position_robot = _center_position_robot;
	_current_rotation_robot = _center_rotation_robot;

	// Initialize scaling factors (can be set through setScalingFactors())
	_scaling_factor_trans=1.0;
	_scaling_factor_rot=1.0;

	//Initialize position controller parameters
	_kp_position_ctrl_device = 0.1;
	_kv_position_ctrl_device = 0.5;
	_kp_orientation_ctrl_device = 0.4;
	_kv_orientation_ctrl_device = 0.2;

	//Initialize force feedback controller parameters
	_proxy_position_impedance = 200.0;
	_proxy_orientation_impedance = 15.0;
	_proxy_position_damping = 10.0;
	_proxy_orientation_damping = 5.0;
	_reduction_factor_torque_feedback << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	_reduction_factor_force_feedback << 1/2.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0,
						  0.0, 0.0, 1/2.0;

	//Initialize sensed force filtering
	_force_filter = new ButterworthFilter(3);
	_moment_filter = new ButterworthFilter(3);

	_cutOff_frequency_force = 0.02;
	_cutOff_frequency_moment = 0.02;
	_force_filter->setCutoffFrequency(_cutOff_frequency_force);
	_moment_filter->setCutoffFrequency(_cutOff_frequency_moment);
	_filter_on = false;

	//Initialiaze force feedback computation mode
	_haptic_feedback_from_proxy = true;
	_send_haptic_feedback = false;

	// To initialize the timer
	_first_iteration = true;

	// Initialize Workspace extension parameters
	_center_position_robot_drift.setZero();
	_center_rotation_robot_drift.setIdentity();
	
	_max_rot_velocity_device=0.001;
	_max_trans_velocity_device=0.001;

	_drift_force.setZero(3);
	_drift_torque.setZero(3);
	_drift_rot_velocity.setZero(3);
	_drift_trans_velocity.setZero(3);

	// Default drift force percentage (can be change with setForceNoticeableDiff())
	_drift_force_admissible_ratio=10.0/100.0;

	// Initialization of the controller parameters
	_device_workspace_radius_max=0.025;
	_task_workspace_radius_max=0.2;
	_device_workspace_tilt_angle_max=20*M_PI/180.0;
	_task_workspace_tilt_angle_max=45*M_PI/180.0;

}



void OpenLoopTeleop::initializeSigmaDevice()
{
	if(device_info.m_modelName == "sigma.7")
	{
		cDeltaDevice* tmp_device = static_cast<cDeltaDevice*>(hapticDevice.get());
		tmp_device->enableForces(true);
	}	
}

OpenLoopTeleop::~OpenLoopTeleop ()
{
	_commanded_force_device.setZero();
	_commanded_torque_device.setZero();
	cVector3d _force_chai;
	cVector3d _torque_chai;
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);	
	hapticDevice->close();

	_desired_position_robot = _center_position_robot;
	_desired_rotation_robot = _center_rotation_robot;

	delete _force_filter;
	_force_filter = NULL;
	delete _moment_filter;
	_moment_filter = NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Core methods ////
void OpenLoopTeleop::computeHapticCommands6d(Eigen::Vector3d& desired_position_robot,
							Eigen::Matrix3d& desired_rotation_robot)
{
	device_homed = false;

	cVector3d _current_position_device_chai;
	cMatrix3d _current_rotation_device_chai;
	// read haptic device position
	hapticDevice->getPosition(_current_position_device_chai);
	_current_position_device = convertChaiToEigenVector(_current_position_device_chai);
	hapticDevice->getRotation(_current_rotation_device_chai);
	_current_rotation_device = convertChaiToEigenMatrix(_current_rotation_device_chai);

	// Compute the force feedback in robot frame
	Vector3d f_task_trans;
	Vector3d f_task_rot;
	Vector3d orientation_dev;
	cVector3d _force_chai;
	cVector3d _torque_chai;
	
	if (_haptic_feedback_from_proxy)
	{
		
		// Evaluate the task force through stiffness proxy
		f_task_trans = _proxy_position_impedance*(_current_position_robot - _desired_position_robot) - _proxy_position_damping * _current_trans_velocity_robot;

		// Compute the orientation error
		Sai2Model::orientationError(orientation_dev, _desired_rotation_robot, _current_rotation_robot);
		// Evaluate task torque
		f_task_rot = _proxy_orientation_impedance*orientation_dev - _proxy_orientation_damping * _current_rot_velocity_robot;

	}
	else
	{
		// Read sensed task force 
		f_task_trans = _sensed_task_force.head(3);
		f_task_rot = _sensed_task_force.tail(3);

	}
	
	// Apply reduction factors to force feedback
	f_task_trans = _reduction_factor_force_feedback * f_task_trans;
	f_task_rot = _reduction_factor_torque_feedback * f_task_rot;
	//Transfer task force from robot to haptic device global frame
	_commanded_force_device = _Rotation_Matrix_DeviceToRobot * f_task_trans;
	_commanded_torque_device = _Rotation_Matrix_DeviceToRobot * f_task_rot;

	// Scaling of the force feedback
	Eigen::Matrix3d scaling_factor_trans;
	Eigen::Matrix3d scaling_factor_rot;

		scaling_factor_trans << 1/_scaling_factor_trans, 0.0, 0.0,
						  0.0, 1/_scaling_factor_trans, 0.0, 
						  0.0, 0.0, 1/_scaling_factor_trans;
		scaling_factor_rot << 1/_scaling_factor_rot, 0.0, 0.0, 
						  0.0, 1/_scaling_factor_rot, 0.0,
						  0.0, 0.0, 1/_scaling_factor_rot;

	_commanded_force_device = scaling_factor_trans * _commanded_force_device;
	_commanded_torque_device = scaling_factor_rot * _commanded_torque_device;

	// Saturate to Force and Torque limits of the haptic device
	if (_commanded_force_device.norm() >= device_info.m_maxLinearForce)
	{
		_commanded_force_device = device_info.m_maxLinearForce*_commanded_force_device/(_commanded_force_device.norm());
	}
	if (_commanded_torque_device.norm() >= device_info.m_maxAngularTorque)
	{
		_commanded_torque_device = device_info.m_maxAngularTorque*_commanded_torque_device/(_commanded_torque_device.norm());
	}

	// Send controllers force and torque to haptic device
	_force_chai = convertEigenToChaiVector(_commanded_force_device);
	_torque_chai = convertEigenToChaiVector(_commanded_torque_device);
	if(!_send_haptic_feedback)
	{
		_force_chai.set(0,0,0);
		_torque_chai.set(0,0,0);
	}
    hapticDevice->setForceAndTorque(_force_chai,_torque_chai);

 	// Compute the set position from the haptic device
	_desired_position_robot = _scaling_factor_trans*(_current_position_device-_home_position_device);
	// Rotation with respect with home orientation

	Eigen::Matrix3d rot_rel = _current_rotation_device * _home_rotation_device.transpose();
	Eigen::AngleAxisd rot_rel_ang = Eigen::AngleAxisd(rot_rel);

	// Compute set orientation from the haptic device
	Eigen::AngleAxisd desired_rotation_robot_aa = Eigen::AngleAxisd(_scaling_factor_rot*rot_rel_ang.angle(),rot_rel_ang.axis());
	_desired_rotation_robot = desired_rotation_robot_aa.toRotationMatrix();

	//Transfer set position and orientation from device to robot global frame
	_desired_position_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_position_robot;
	_desired_rotation_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_rotation_robot * _Rotation_Matrix_DeviceToRobot * _center_rotation_robot; 

	// Adjust set position to the center of the task workspace
	_desired_position_robot = _desired_position_robot + _center_position_robot;

	// Send set position orientation of the robot
	desired_position_robot = _desired_position_robot;
	desired_rotation_robot = _desired_rotation_robot;
}


void OpenLoopTeleop::computeHapticCommands3d(Eigen::Vector3d& desired_position_robot)
{
	device_homed = false;

	cVector3d _current_position_device_chai;
	cMatrix3d _current_rotation_device_chai;
	// read haptic device position
	hapticDevice->getPosition(_current_position_device_chai);
	_current_position_device = convertChaiToEigenVector(_current_position_device_chai);

	// Compute the force feedback in robot frame
	Vector3d f_task_trans;
	Vector3d f_task_rot;
	Vector3d orientation_dev;
	cVector3d _force_chai;
	cVector3d _torque_chai;
	
	if (_haptic_feedback_from_proxy)
	{
		// Evaluate the task force through stiffness proxy
		f_task_trans = _proxy_position_impedance*(_current_position_robot - _desired_position_robot) - _proxy_position_damping * _current_trans_velocity_robot;
		f_task_rot.setZero();
	}
	else
	{
		// Read sensed task force 
		f_task_trans = _sensed_task_force.head(3);
		f_task_rot.setZero();
	}
	
	// Apply reduction factors to force feedback
	f_task_trans = _reduction_factor_force_feedback * f_task_trans;
	f_task_rot = _reduction_factor_torque_feedback * f_task_rot;
	//Transfer task force from robot to haptic device global frame
	_commanded_force_device = _Rotation_Matrix_DeviceToRobot * f_task_trans;
	_commanded_torque_device = _Rotation_Matrix_DeviceToRobot * f_task_rot;

	// Scaling of the force feedback
	Eigen::Matrix3d scaling_factor_trans;
	Eigen::Matrix3d scaling_factor_rot;

		scaling_factor_trans << 1/_scaling_factor_trans, 0.0, 0.0,
						  0.0, 1/_scaling_factor_trans, 0.0, 
						  0.0, 0.0, 1/_scaling_factor_trans;
		scaling_factor_rot << 1/_scaling_factor_rot, 0.0, 0.0, 
						  0.0, 1/_scaling_factor_rot, 0.0,
						  0.0, 0.0, 1/_scaling_factor_rot;

	_commanded_force_device = scaling_factor_trans * _commanded_force_device;
	_commanded_torque_device = scaling_factor_rot * _commanded_torque_device;

	// Saturate to Force and Torque limits of the haptic device
	if (_commanded_force_device.norm() >= device_info.m_maxLinearForce)
	{
		_commanded_force_device = device_info.m_maxLinearForce*_commanded_force_device/(_commanded_force_device.norm());
	}
	if (_commanded_torque_device.norm() >= device_info.m_maxAngularTorque)
	{
		_commanded_torque_device = device_info.m_maxAngularTorque*_commanded_torque_device/(_commanded_torque_device.norm());
	}

	// Send controllers force and torque to haptic device
	_force_chai = convertEigenToChaiVector(_commanded_force_device);
	_torque_chai = convertEigenToChaiVector(_commanded_torque_device);
	if(!_send_haptic_feedback)
	{
		_force_chai.set(0,0,0);
		_torque_chai.set(0,0,0);
	}
    hapticDevice->setForceAndTorque(_force_chai,_torque_chai);


 	// Compute the set position from the haptic device
	_desired_position_robot = _scaling_factor_trans*(_current_position_device-_home_position_device);
	// Rotation with respect with home orientation
	
	//Transfer set position and orientation from device to robot global frame
	_desired_position_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_position_robot;
	
	// Adjust set position to the center of the task Workspace
	_desired_position_robot = _desired_position_robot + _center_position_robot;

	// Send set position orientation of the robot
	desired_position_robot = _desired_position_robot;
	

}




void OpenLoopTeleop::computeHapticCommandsWorkspaceExtension6d(Eigen::Vector3d& desired_position_robot,
							Eigen::Matrix3d& desired_rotation_robot)
{
	// get time since last call
	 _t_curr = std::chrono::high_resolution_clock::now();
	 if(_first_iteration)
	 {
	 	_t_prev = std::chrono::high_resolution_clock::now();
	 	_first_iteration = false;
	 	_desired_position_robot = _center_position_robot;
	 	_desired_rotation_robot = _center_rotation_robot;
	 	_center_position_robot_drift = _center_position_robot;
	 	_center_rotation_robot_drift = _center_rotation_robot;
	 }
	 _t_diff = _t_curr - _t_prev;

	device_homed = false;

 	cVector3d _current_position_device_chai;
	cMatrix3d _current_rotation_device_chai;
 	cVector3d _current_trans_velocity_device_chai;
 	cVector3d _current_rot_velocity_device_chai;
	// read haptic device position and velocity
	hapticDevice->getPosition(_current_position_device_chai);
	hapticDevice->getLinearVelocity(_current_trans_velocity_device_chai);
	_current_position_device = convertChaiToEigenVector(_current_position_device_chai);
	_current_trans_velocity_device = convertChaiToEigenVector(_current_trans_velocity_device_chai);
	
	hapticDevice->getRotation(_current_rotation_device_chai);												//
	_current_rotation_device = convertChaiToEigenMatrix(_current_rotation_device_chai);						//
	hapticDevice->getAngularVelocity(_current_rot_velocity_device_chai);									//
	_current_rot_velocity_device = convertChaiToEigenVector(_current_rot_velocity_device_chai);				//
	

	// Update the maximum velocities for the task
	if (_current_rot_velocity_device.norm()>=_max_rot_velocity_device)	
	{
		_max_rot_velocity_device = _current_rot_velocity_device.norm();
    }

	if (_current_trans_velocity_device.norm()>=_max_trans_velocity_device)	
	{
		_max_trans_velocity_device = _current_trans_velocity_device.norm();
	}

	// Compute the force feedback in robot frame
	Vector3d f_task_trans;
	Vector3d f_task_rot;
	Vector3d orientation_dev;
	cVector3d _force_chai;
	cVector3d _torque_chai;
	
	if (_haptic_feedback_from_proxy)
	{
		// Evaluate the task force through stiffness proxy
		f_task_trans = _proxy_position_impedance*(_current_position_robot - _desired_position_robot) - _proxy_position_damping * _current_trans_velocity_robot;
		
		// Compute the orientation error
		Sai2Model::orientationError(orientation_dev, _desired_rotation_robot, _current_rotation_robot);							//
		// Evaluate task torque
		f_task_rot = _proxy_orientation_impedance*orientation_dev - _proxy_orientation_damping * _current_rot_velocity_robot;	//
		
		// f_task_rot.setZero();
		
	}
	else
	{
		// Read sensed task force 
		f_task_trans = _sensed_task_force.head(3);
		f_task_rot = _sensed_task_force.tail(3);																				//
	
		// f_task_rot.setZero();
	
	}

	// Apply reduction factors to force feedback
	f_task_trans = _reduction_factor_force_feedback * f_task_trans;
	f_task_rot = _reduction_factor_torque_feedback * f_task_rot;
	//Transfer task force from robot to haptic device global frame
	_commanded_force_device = _Rotation_Matrix_DeviceToRobot * f_task_trans;
	_commanded_torque_device = _Rotation_Matrix_DeviceToRobot * f_task_rot;


	//// Evaluation of the drift velocities ////
	//Translational drift
	Vector3d relative_position_device;
	relative_position_device = _current_position_device-_home_position_device;
	_drift_trans_velocity = -_current_trans_velocity_device.norm()*relative_position_device/(_device_workspace_radius_max*_max_trans_velocity_device);
	// Rotational drift
	Matrix3d relative_rotation_device = _current_rotation_device * _home_rotation_device.transpose(); // Rotation with respect with home orientation
	AngleAxisd relative_orientation_angle_device = AngleAxisd(relative_rotation_device);
	_drift_rot_velocity=-(_current_rot_velocity_device.norm()*relative_orientation_angle_device.angle()*relative_orientation_angle_device.axis())/(_device_workspace_tilt_angle_max*_max_rot_velocity_device);

	//// Computation of the scaling factors ////
	_scaling_factor_trans = 1.0 + relative_position_device.norm()*(_task_workspace_radius_max/_device_workspace_radius_max-1.0)/_device_workspace_radius_max;
	_scaling_factor_rot = 1.0 + relative_orientation_angle_device.angle()*(_task_workspace_tilt_angle_max/_device_workspace_tilt_angle_max-1.0)/_device_workspace_tilt_angle_max;
	
	// Scaling of the force feedback
	Eigen::Matrix3d scaling_factor_trans;
	Eigen::Matrix3d scaling_factor_rot;
		scaling_factor_trans << 1/_scaling_factor_trans, 0.0, 0.0,
						  0.0, 1/_scaling_factor_trans, 0.0, 
						  0.0, 0.0, 1/_scaling_factor_trans;
		scaling_factor_rot << 1/_scaling_factor_rot, 0.0, 0.0, 
						  0.0, 1/_scaling_factor_rot, 0.0,
						  0.0, 0.0, 1/_scaling_factor_rot;

	_commanded_force_device = scaling_factor_trans * _commanded_force_device;
	_commanded_torque_device = scaling_factor_rot * _commanded_torque_device;
	
	//// Evaluation of the drift force ////
	// Definition of the velocity gains from task feedback
	Matrix3d _Kv_translation = _drift_force_admissible_ratio*(_commanded_force_device.asDiagonal());
	Matrix3d _Kv_rotation = _drift_force_admissible_ratio*(_commanded_torque_device.asDiagonal());

	// Drift force computation
	_drift_force = _Kv_translation * _drift_trans_velocity;
	_drift_torque = _Kv_rotation * _drift_rot_velocity;
	//_drift_force = Lambda*_drift_force_0; //Drift force weigthed through the device inertia matrix

	//// Desired cartesian force to apply to the haptic device ////
	_commanded_force_device += _drift_force;
	_commanded_torque_device += _drift_torque;

	//// Add virtual forces according to the device operational space limits ////
	Vector3d force_virtual = Vector3d::Zero();
	Vector3d torque_virtual = Vector3d::Zero();

	if (relative_position_device.norm() >= _device_workspace_radius_max)
	{
		force_virtual = -(0.8 * device_info.m_maxLinearStiffness * (relative_position_device.norm()-_device_workspace_radius_max)/(relative_position_device.norm()))*relative_position_device;
	}
	_commanded_force_device += force_virtual;
	
	if (relative_orientation_angle_device.angle() >= _device_workspace_tilt_angle_max)
	{
		torque_virtual = -(0.8 * device_info.m_maxAngularStiffness *(relative_orientation_angle_device.angle()-_device_workspace_tilt_angle_max))*relative_orientation_angle_device.axis();
	}
	_commanded_torque_device += torque_virtual;

	// Saturate to Force and Torque limits of the haptic device
	if (_commanded_force_device.norm() >= device_info.m_maxLinearForce)
	{
		_commanded_force_device = device_info.m_maxLinearForce*_commanded_force_device/(_commanded_force_device.norm());
	}
	if (_commanded_torque_device.norm() >= device_info.m_maxAngularTorque)
	{
		_commanded_torque_device = device_info.m_maxAngularTorque*_commanded_torque_device/(_commanded_torque_device.norm());
	}

	// Send controllers force and torque to haptic device
	_force_chai = convertEigenToChaiVector(_commanded_force_device);
	_torque_chai = convertEigenToChaiVector(_commanded_torque_device);
	if(!_send_haptic_feedback)
	{
		_force_chai.set(0,0,0);
		_torque_chai.set(0,0,0);
	}
    hapticDevice->setForceAndTorque(_force_chai,_torque_chai);

	//// Computation of the desired position for the controlled robot after drift of the device ////
	// Estimated drift velocity considering drift force 
	// VectorXd _vel_drift_est = _t_diff.count()*Lambda.inverse()*_Fdrift; // if estimated human+device mass matrix
	Vector3d _vel_drift_est_trans = _t_diff.count()*_drift_force;
	Vector3d _vel_drift_est_rot = _t_diff.count()*_drift_torque;

	// Drift of the center of the task workspace
	_center_position_robot_drift -= _Rotation_Matrix_DeviceToRobot.transpose()*(_t_diff.count()*_scaling_factor_trans*_vel_drift_est_trans);

	double _centerRot_angle = -_t_diff.count()*_scaling_factor_rot*_vel_drift_est_rot.norm();
	Vector3d _centerRot_axis;
	if (abs(_centerRot_angle) >= 0.00001)
	{
		_centerRot_axis = _vel_drift_est_rot/_vel_drift_est_rot.norm();
		AngleAxisd _centerRot_angleAxis=AngleAxisd(_centerRot_angle,_centerRot_axis);

		_center_rotation_robot_drift = _Rotation_Matrix_DeviceToRobot.transpose()*(_centerRot_angleAxis.toRotationMatrix())*_Rotation_Matrix_DeviceToRobot*_center_rotation_robot_drift;
	}

	//// Compute position of the controlled robot after drift ////
	 	// Compute the set position from the haptic device
	_desired_position_robot = _scaling_factor_trans*relative_position_device;
	// Rotation with respect with home orientation
	
	// Compute set orientation from the haptic device
	Eigen::AngleAxisd desired_rotation_robot_aa = Eigen::AngleAxisd(_scaling_factor_rot*relative_orientation_angle_device.angle(),relative_orientation_angle_device.axis());			//
	_desired_rotation_robot = desired_rotation_robot_aa.toRotationMatrix();					//
	

	//Transfer set position and orientation from device to robot global frame
	_desired_position_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_position_robot;
	_desired_rotation_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_rotation_robot * _Rotation_Matrix_DeviceToRobot * _center_rotation_robot_drift; 					//

	// Adjust set position to the center of the task Workspace
	_desired_position_robot = _desired_position_robot + _center_position_robot_drift;

	// Send set position orientation of the robot
	desired_position_robot = _desired_position_robot;
	desired_rotation_robot = _desired_rotation_robot;										//

	// update previous time
	 _t_prev = _t_curr;
}








void OpenLoopTeleop::updateSensedForce(const Eigen::VectorXd sensed_task_force)
{
	if (_filter_on)
	{
		Vector3d f_task_trans_sensed = sensed_task_force.head(3);
		Vector3d f_task_rot_sensed = sensed_task_force.tail(3);
		f_task_trans_sensed = _force_filter->update(f_task_trans_sensed);
		f_task_rot_sensed = _moment_filter->update(f_task_rot_sensed);

		_sensed_task_force << f_task_trans_sensed, f_task_rot_sensed;
	}
	else
	{
		_sensed_task_force = sensed_task_force;
	}
}

void OpenLoopTeleop::updateSensedRobotPositionVelocity(const Eigen::Vector3d current_position_robot,
								const Eigen::Vector3d current_trans_velocity_robot,
								const Eigen::Matrix3d current_rotation_robot,
								const Eigen::Vector3d current_rot_velocity_robot)
{
	_current_position_robot = current_position_robot;
	_current_rotation_robot = current_rotation_robot;
	_current_trans_velocity_robot = current_trans_velocity_robot;
	_current_rot_velocity_robot = current_rot_velocity_robot;

}


void OpenLoopTeleop::GravityCompTask()
{	
	_commanded_force_device.setZero();
	_commanded_torque_device.setZero();
	cVector3d _force_chai;
	cVector3d _torque_chai;
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);
}

void OpenLoopTeleop::HomingTask()
{	
	
	cVector3d _force_chai;
	cVector3d _torque_chai;
	cVector3d _current_position_device_chai;
	cMatrix3d _current_rotation_device_chai;
	cVector3d _current_trans_velocity_device_chai;
	cVector3d _current_rot_velocity_device_chai;
	// Haptice device position controller gains
	double kp_position_ctrl_device =_kp_position_ctrl_device * device_info.m_maxLinearStiffness;
	double kv_position_ctrl_device =_kv_position_ctrl_device * device_info.m_maxLinearDamping;
	double kp_orientation_ctrl_device =_kp_orientation_ctrl_device * device_info.m_maxAngularStiffness;
	double kv_orientation_ctrl_device =_kv_orientation_ctrl_device * device_info.m_maxAngularDamping;

	// read haptic device position and velocity
	hapticDevice->getPosition(_current_position_device_chai);
	hapticDevice->getRotation(_current_rotation_device_chai);
	_current_position_device = convertChaiToEigenVector(_current_position_device_chai);
	_current_rotation_device = convertChaiToEigenMatrix(_current_rotation_device_chai);
	hapticDevice->getLinearVelocity(_current_trans_velocity_device_chai);
	hapticDevice->getAngularVelocity(_current_rot_velocity_device_chai);
	_current_trans_velocity_device = convertChaiToEigenVector(_current_trans_velocity_device_chai);
	_current_rot_velocity_device = convertChaiToEigenVector(_current_rot_velocity_device_chai);

	// Evaluate position controller force
	_commanded_force_device = -kp_position_ctrl_device*(_current_position_device - _home_position_device) - kv_position_ctrl_device * _current_trans_velocity_device;
	// Compute the orientation error
	Vector3d orientation_error;
	Sai2Model::orientationError(orientation_error, _home_rotation_device, _current_rotation_device);
	// Evaluate orientation controller force
	_commanded_torque_device = -kp_orientation_ctrl_device*orientation_error - kv_orientation_ctrl_device * _current_rot_velocity_device;

	// Saturate to Force and Torque limits of the haptic device
	if (_commanded_force_device.norm() >= device_info.m_maxLinearForce)
	{
		_commanded_force_device = device_info.m_maxLinearForce*_commanded_force_device/(_commanded_force_device.norm());
	}
	if (_commanded_torque_device.norm() >= device_info.m_maxAngularTorque)
	{
		_commanded_torque_device = device_info.m_maxAngularTorque*_commanded_torque_device/(_commanded_torque_device.norm());
	}

	// Send controllers force and torque to haptic device
	_force_chai = convertEigenToChaiVector(_commanded_force_device);
	_torque_chai = convertEigenToChaiVector(_commanded_torque_device);
    hapticDevice->setForceAndTorque(_force_chai,_torque_chai);
    //&& orientation_error.norm() < 0.04
	if( (_current_position_device - _home_position_device).norm()<0.002)
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
	cVector3d _force_chai;
	cVector3d _torque_chai;
	//Send zero force feedback to the haptic device
	_commanded_force_device.setZero();
	_commanded_torque_device.setZero();
	_force_chai.set(0.0,0.0,0.0);
	_torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(_force_chai,_torque_chai);

	//reInitialize snesed force to zero
	_sensed_task_force.setZero(6);

	//reInitialize homing task
	device_homed = false;
	// Default home position
	_home_position_device.setZero();
	_home_rotation_device.setIdentity();

	//reInitialize Workspace center of the controlled robot to defaut values
	_center_position_robot.setZero(); 
	_center_rotation_robot.setIdentity();

	//Initialize the set position and orientation of the controlled robot
	_desired_position_robot = _center_position_robot;
	_desired_rotation_robot = _center_rotation_robot;
	_current_position_robot = _center_position_robot;
	_current_rotation_robot = _center_rotation_robot;

	//Initialize the frame trasform from device to robot
	_Rotation_Matrix_DeviceToRobot.setIdentity();

	//reInitialize scaling factors to defaut values
	_scaling_factor_trans=1.0;
	_scaling_factor_rot=1.0;

	//Initialize position controller parameters
	_kp_position_ctrl_device = 0.1;
	_kv_position_ctrl_device = 0.5;
	_kp_orientation_ctrl_device = 0.4;
	_kv_orientation_ctrl_device = 0.2;

	//Initialize force feedback controller parameters
	_proxy_position_impedance = 200.0;
	_proxy_orientation_impedance = 15.0;
	_proxy_position_damping = 10.0;
	_proxy_orientation_damping = 5.0;
	_reduction_factor_torque_feedback << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	_reduction_factor_force_feedback << 1/2.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0,
						  0.0, 0.0, 1/2.0;

	//reInitialize filter parameters
	_cutOff_frequency_force = 0.02;
	_cutOff_frequency_moment = 0.02;
	_force_filter->setCutoffFrequency(_cutOff_frequency_force);
	_moment_filter->setCutoffFrequency(_cutOff_frequency_moment);
	_filter_on = false;

	//Initialiaze force feedback computation mode
	_haptic_feedback_from_proxy = true;
	_send_haptic_feedback = false;

	_first_iteration = true; // To initialize the timer

	// Initialize Workspace extension parameters
	_center_position_robot_drift.setZero();
	_center_rotation_robot_drift.setIdentity();

	_max_rot_velocity_device=0.001;
	_max_trans_velocity_device=0.001;

	_drift_force.setZero(3);
	_drift_torque.setZero(3);
	_drift_rot_velocity.setZero(3);
	_drift_trans_velocity.setZero(3);


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Parameter setting methods ////

void OpenLoopTeleop::setScalingFactors(const double scaling_factor_trans, const double scaling_factor_rot)
{
	_scaling_factor_trans = scaling_factor_trans;
	_scaling_factor_rot = scaling_factor_rot;
}

void OpenLoopTeleop::setPosCtrlGains (const double kp_position_ctrl_device, const double kv_position_ctrl_device, const double kp_orientation_ctrl_device, const double kv_orientation_ctrl_device)
{
	_kp_position_ctrl_device = kp_position_ctrl_device;
	_kv_position_ctrl_device = kv_position_ctrl_device;
	_kp_orientation_ctrl_device = kp_orientation_ctrl_device;
	_kv_orientation_ctrl_device = kv_orientation_ctrl_device;
}

void OpenLoopTeleop::setForceFeedbackCtrlGains (const double proxy_position_impedance, const double proxy_position_damping, const double proxy_orientation_impedance, const double proxy_orientation_damping,
		const Matrix3d reduction_factor_torque_feedback,
		const Matrix3d reduction_factor_force_feedback)
{
	_proxy_position_impedance = proxy_position_impedance;
	_proxy_orientation_impedance = proxy_orientation_impedance;
	_proxy_position_damping = proxy_position_damping;
	_proxy_orientation_damping = proxy_orientation_damping;
	_reduction_factor_torque_feedback = reduction_factor_torque_feedback;
	_reduction_factor_force_feedback = reduction_factor_force_feedback;
}

void OpenLoopTeleop::setFilterCutOffFreq(const double cutOff_frequency_force, const double cutOff_frequency_moment)
{
	_cutOff_frequency_force = cutOff_frequency_force;
	_cutOff_frequency_moment = cutOff_frequency_moment;
	_force_filter->setCutoffFrequency(_cutOff_frequency_force);
	_moment_filter->setCutoffFrequency(_cutOff_frequency_moment);

}

void OpenLoopTeleop::setDeviceCenter(const Eigen::Vector3d home_position_device, const Eigen::Matrix3d home_rotation_device)
{
	_home_position_device = home_position_device;
	_home_rotation_device = home_rotation_device;
}

void OpenLoopTeleop::setDeviceOrientationCenterToCurrent()
{
	cMatrix3d device_orientation_chai;
	hapticDevice->getRotation(device_orientation_chai);
	_home_rotation_device = device_orientation_chai.eigen();
}

void OpenLoopTeleop::setDevicePositionCenterToCurrent()
{
	cVector3d device_position_chai;
	hapticDevice->getPosition(device_position_chai);
	_home_position_device = device_position_chai.eigen();
}

void OpenLoopTeleop::setRobotCenter(const Eigen::Vector3d center_position_robot, const Eigen::Matrix3d center_rotation_robot)
{
	_center_position_robot = center_position_robot; 
	_center_rotation_robot = center_rotation_robot;

	//Initialize the set position and orientation of the controlled robot
	_desired_position_robot = _center_position_robot;
	_desired_rotation_robot = _center_rotation_robot;

}

void OpenLoopTeleop::setDeviceRobotRotation(const Eigen::Matrix3d Rotation_Matrix_DeviceToRobot)
{
	_Rotation_Matrix_DeviceToRobot = Rotation_Matrix_DeviceToRobot;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Workspace extension related methods ////
void OpenLoopTeleop::setWorkspaceSize(double device_workspace_radius_max, double task_workspace_radius_max, double device_workspace_tilt_angle_max, double task_workspace_tilt_angle_max)
{
	_device_workspace_radius_max = device_workspace_radius_max;
	_task_workspace_radius_max = task_workspace_radius_max;
	_device_workspace_tilt_angle_max = device_workspace_tilt_angle_max;
	_task_workspace_tilt_angle_max = task_workspace_tilt_angle_max;
}


void OpenLoopTeleop::setForceNoticeableDiff(double drift_force_admissible_ratio)
{
	_drift_force_admissible_ratio = drift_force_admissible_ratio;
}



} /* namespace Sai2Primitives */

