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

////////////////////////////////////////////////////////////////////////////////////////////////////
//// Declaration of internal functions
////////////////////////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////////////////////////
//// Constructor, Destructor and Initialization of the haptic controllers
////////////////////////////////////////////////////////////////////////////////////////////////////

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

	//Initialize the set force and torque of the controlled robot
	_desired_force_robot.setZero();
	_desired_torque_robot.setZero();

	//Initialize the set velocity of the controlled robot
	_desired_trans_velocity_robot.setZero();
	_desired_rot_velocity_robot.setZero();
	_integrated_trans_velocity_error.setZero();
	_integrated_rot_velocity_error.setZero();

	//Set the initial robot position, orientation, and velocity
	_current_position_robot = _center_position_robot;
	_current_rotation_robot = _center_rotation_robot;
	_current_trans_velocity_robot.setZero();
	_current_rot_velocity_robot.setZero();
	//Set the initial virtual proxy position, orientation, and velocity
	_current_position_proxy = _center_position_robot;
	_current_rotation_proxy = _center_rotation_robot;
	_current_trans_velocity_proxy.setZero();
	_current_rot_velocity_proxy.setZero();

	// Initialize scaling factors (can be set through setScalingFactors())
	_scaling_factor_trans=1.0;
	_scaling_factor_rot=1.0;

	//Initialize position controller parameters
	_kp_position_ctrl_device = 0.2;
	_kv_position_ctrl_device = 0.7;
	_kp_orientation_ctrl_device = 0.6;
	_kv_orientation_ctrl_device = 0.2;

	//Initialize virtual proxy parameters
	_proxy_position_impedance = 1000.0;
	_proxy_orientation_impedance = 10.0;
	_proxy_position_damping = 20.0;
	_proxy_orientation_damping = 0.5;

	//Initialize force feedback controller parameters
	_kp_robot_trans_velocity = 10.0;
	_kp_robot_rot_velocity = 10.0;
	_ki_robot_trans_velocity = 0.0;
	_ki_robot_rot_velocity = 0.0;

	_robot_trans_admittance = 1/50.0;
	_robot_rot_admittance = 1/1.5;

	_reduction_factor_torque_feedback << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	_reduction_factor_force_feedback << 1/2.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0,
						  0.0, 0.0, 1/2.0;

	// Initialize virtual force guidance gains for unified controller
	_force_guidance_position_impedance = 200.0;
	_force_guidance_orientation_impedance = 20.0;
	_force_guidance_position_damping = 40.0;
	_force_guidance_orientation_damping = 2.0;

	// Set selection matrices to full motion control
	_sigma_position.setIdentity();
	_sigma_orientation.setIdentity();
	_sigma_force.setZero();
	_sigma_moment.setZero();

	//Initialize sensed force filtering
	_force_filter = new ButterworthFilter(3);
	_moment_filter = new ButterworthFilter(3);

	_cutOff_frequency_force = 0.04;
	_cutOff_frequency_moment = 0.04;
	_force_filter->setCutoffFrequency(_cutOff_frequency_force);
	_moment_filter->setCutoffFrequency(_cutOff_frequency_moment);
	_filter_on = false;

	//Initialiaze force feedback computation mode
	_haptic_feedback_from_proxy = false;
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
	_drift_force_admissible_ratio = 50.0/100.0;
	_drift_velocity_admissible_ratio = 50.0/100.0;

	// Initialization of the controller parameters
	_device_workspace_radius_max=0.025;
	_task_workspace_radius_max=0.2;
	_device_workspace_tilt_angle_max=20*M_PI/180.0;
	_task_workspace_tilt_angle_max=45*M_PI/180.0;

	// Device workspace virtual limits
	_add_workspace_virtual_limit=false;
	_device_workspace_radius_limit = 0.1;
	_device_workspace_angle_limit = 90*M_PI/180.0;

	//Initialize haptic guidance parameters
	_enable_plane_guidance_3D=false;
	_enable_line_guidance_3D=false;
	_guidance_stiffness=0.7;
	_guidance_damping=0.8;
	_guidance_force_plane.setZero();
	_guidance_force_line.setZero();

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
	_desired_trans_velocity_robot.setZero();
	_desired_rot_velocity_robot.setZero();
	_integrated_trans_velocity_error.setZero();
	_integrated_rot_velocity_error.setZero();

	delete _force_filter;
	_force_filter = NULL;
	delete _moment_filter;
	_moment_filter = NULL;
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

	//Initialize the set position and orientation of the controlled robot
	_desired_position_robot = _current_position_robot;
	_desired_rotation_robot = _current_rotation_robot;

	//Initialize the set force and torque of the controlled robot
	_desired_force_robot.setZero();
	_desired_torque_robot.setZero();

	//Initialize the set velocity of the controlled robot
	_desired_trans_velocity_robot.setZero();
	_desired_rot_velocity_robot.setZero();
	_integrated_trans_velocity_error.setZero();
	_integrated_rot_velocity_error.setZero();
	
	_first_iteration = true; // To initialize the timer and integral terms

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

////////////////////////////////////////////////////////////////////////////////////////////////////
// Impedance controllers in bilateral teleoperation scheme
////////////////////////////////////////////////////////////////////////////////////////////////////
void OpenLoopTeleop::computeHapticCommands6d(Eigen::Vector3d& desired_position_robot,
							Eigen::Matrix3d& desired_rotation_robot)
{
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
	
	hapticDevice->getRotation(_current_rotation_device_chai);
	_current_rotation_device = convertChaiToEigenMatrix(_current_rotation_device_chai);	
	hapticDevice->getAngularVelocity(_current_rot_velocity_device_chai);
	_current_rot_velocity_device = convertChaiToEigenVector(_current_rot_velocity_device_chai);	

	//Transfer device velocity to robot global frame
	_current_trans_velocity_device_RobFrame = _scaling_factor_trans * _Rotation_Matrix_DeviceToRobot.transpose() * _current_trans_velocity_device;
	_current_rot_velocity_device_RobFrame = _scaling_factor_rot * _Rotation_Matrix_DeviceToRobot.transpose() * _current_rot_velocity_device; 


	if(_first_iteration)
	 {
	 	_first_iteration = false;
	 	// Set the initial desired position to the robot center
	 	_desired_position_robot = _current_position_robot;
	 	_desired_rotation_robot = _current_rotation_robot;
	 }

		// Position of the device with respect with home position
	Vector3d relative_position_device;
	relative_position_device = _current_position_device-_home_position_device;
	// Rotation of the device with respect with home orientation
	Matrix3d relative_rotation_device = _current_rotation_device * _home_rotation_device.transpose(); // Rotation with respect with home orientation
	AngleAxisd relative_orientation_angle_device = AngleAxisd(relative_rotation_device);

	// Compute the force feedback in robot frame
	Vector3d f_task_trans;
	Vector3d f_task_rot;
	Vector3d orientation_dev;
	cVector3d _force_chai;
	cVector3d _torque_chai;
	
	if (_haptic_feedback_from_proxy)
	{	
		// Evaluate the task force through stiffness proxy
		f_task_trans = _proxy_position_impedance*(_current_position_proxy - _desired_position_robot) - _proxy_position_damping * (_current_trans_velocity_device_RobFrame - _current_trans_velocity_proxy);

		// Compute the orientation error
		Sai2Model::orientationError(orientation_dev, _desired_rotation_robot, _current_rotation_proxy);
		// Evaluate task torque
		f_task_rot = _proxy_orientation_impedance*orientation_dev - _proxy_orientation_damping * (_current_rot_velocity_device_RobFrame - _current_rot_velocity_proxy);

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

	// Apply haptic guidances if activated
	if (_enable_plane_guidance_3D)
	{
		ComputePlaneGuidanceForce();
		_commanded_force_device += _guidance_force_plane - _commanded_force_device.dot(_plane_normal_vec) * _plane_normal_vec;
	}

	if (_enable_line_guidance_3D)
	{
		ComputeLineGuidanceForce();
		Vector3d line_vector = (_line_first_point - _line_second_point) / (_line_first_point - _line_second_point).norm();
		_commanded_force_device = _guidance_force_line + _commanded_force_device.dot(line_vector) * line_vector;
	}

	if(_add_workspace_virtual_limit)
	{
		// Add virtual forces according to the device operational space limits
		Vector3d force_virtual = Vector3d::Zero();
		Vector3d torque_virtual = Vector3d::Zero();

		if (relative_position_device.norm() >= _device_workspace_radius_limit)
		{
			force_virtual = -(0.8 * device_info.m_maxLinearStiffness * (relative_position_device.norm()-_device_workspace_radius_limit)/(relative_position_device.norm()))*relative_position_device;
		}
		_commanded_force_device += force_virtual;
		
		if (relative_orientation_angle_device.angle() >= _device_workspace_angle_limit)
		{
			torque_virtual = -(0.8 * device_info.m_maxAngularStiffness *(relative_orientation_angle_device.angle()-_device_workspace_angle_limit))*relative_orientation_angle_device.axis();
		}
		_commanded_torque_device += torque_virtual;		
	}

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
	_desired_position_robot = _scaling_factor_trans*relative_position_device;
	
	// Rotation with respect with home orientation
	Eigen::Matrix3d rot_rel = _current_rotation_device * _home_rotation_device.transpose();
	Eigen::AngleAxisd rot_rel_ang = Eigen::AngleAxisd(rot_rel);
	// Compute set orientation from the haptic device
	Eigen::AngleAxisd desired_rotation_robot_aa = Eigen::AngleAxisd(_scaling_factor_rot*relative_orientation_angle_device.angle(),relative_orientation_angle_device.axis());
	_desired_rotation_robot = desired_rotation_robot_aa.toRotationMatrix();

	//Transfer set position and orientation from device to robot global frame
	_desired_position_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_position_robot;
	_desired_rotation_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_rotation_robot * _Rotation_Matrix_DeviceToRobot * _center_rotation_robot; 

	// Adjust set position to the center of the task workspace
	_desired_position_robot = _desired_position_robot + _center_position_robot;

    // Send set position and orientation to the robot
	desired_position_robot = _desired_position_robot;
	desired_rotation_robot = _desired_rotation_robot;
}


void OpenLoopTeleop::computeHapticCommands3d(Eigen::Vector3d& desired_position_robot)
{
	device_homed = false;


	cVector3d _current_position_device_chai;
 	cVector3d _current_trans_velocity_device_chai;
	// read haptic device position and velocity
	hapticDevice->getPosition(_current_position_device_chai);
	hapticDevice->getLinearVelocity(_current_trans_velocity_device_chai);
	_current_position_device = convertChaiToEigenVector(_current_position_device_chai);
	_current_trans_velocity_device = convertChaiToEigenVector(_current_trans_velocity_device_chai);

	//Transfer device velocity to robot global frame
	_current_trans_velocity_device_RobFrame = _scaling_factor_trans * _Rotation_Matrix_DeviceToRobot.transpose() * _current_trans_velocity_device;
	
	// Position of the device with respect with home position
	Vector3d relative_position_device;
	relative_position_device = _current_position_device-_home_position_device;
	

	if(_first_iteration)
	 {
	 	_first_iteration = false;
	 	// Set the initial desired position to the robot center
	 	_desired_position_robot = _current_position_robot;
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
		f_task_trans = _proxy_position_impedance*(_current_position_proxy - _desired_position_robot) - _proxy_position_damping * (_current_trans_velocity_device_RobFrame - _current_trans_velocity_proxy);
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

	// Apply haptic guidance if activated
	if (_enable_plane_guidance_3D)
	{
		ComputePlaneGuidanceForce();
		_commanded_force_device += _guidance_force_plane - _commanded_force_device.dot(_plane_normal_vec) * _plane_normal_vec;
	}

	if (_enable_line_guidance_3D)
	{
		ComputeLineGuidanceForce();
		Vector3d line_vector = (_line_first_point - _line_second_point) / (_line_first_point - _line_second_point).norm();
		_commanded_force_device = _guidance_force_line + _commanded_force_device.dot(line_vector) * line_vector;
	}

	if(_add_workspace_virtual_limit)
	{
		// Add virtual forces according to the device operational space limits
		Vector3d force_virtual = Vector3d::Zero();
		if (relative_position_device.norm() >= _device_workspace_radius_limit)
		{
			force_virtual = -(0.8 * device_info.m_maxLinearStiffness * (relative_position_device.norm()-_device_workspace_radius_limit)/(relative_position_device.norm()))*relative_position_device;
		}
		_commanded_force_device += force_virtual;		
	}

	// Saturate to Force and Torque limits of the haptic device
	if (_commanded_force_device.norm() >= device_info.m_maxLinearForce)
	{
		_commanded_force_device = device_info.m_maxLinearForce*_commanded_force_device/(_commanded_force_device.norm());
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
	_desired_position_robot = _scaling_factor_trans*relative_position_device;
	// Rotation with respect with home orientation
	
	//Transfer set position and orientation from device to robot global frame
	_desired_position_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_position_robot;
	
	// Adjust set position to the center of the task Workspace
	_desired_position_robot = _desired_position_robot + _center_position_robot;

    // Send set position to the robot
	desired_position_robot = _desired_position_robot;

}



////////////////////////////////////////////////////////////////////////////////////////////////////
// Admittance controllers in bilateral teleoperation scheme
////////////////////////////////////////////////////////////////////////////////////////////////////
void OpenLoopTeleop::computeHapticCommandsAdmittance6d(Eigen::Vector3d& desired_trans_velocity_robot,
							Eigen::Vector3d& desired_rot_velocity_robot)
{
	device_homed = false;

	// get time since last call for the I term
	_t_curr = std::chrono::high_resolution_clock::now();
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
		_integrated_trans_velocity_error.setZero();
		_integrated_rot_velocity_error.setZero();
	}
	_t_diff = _t_curr - _t_prev;

    // Update sensed force from the haptic device (if no force sensor the method considers the commanded forces)
	cVector3d _sensed_force_device_chai;
 	cVector3d _sensed_torque_device_chai;
	hapticDevice->getForce(_sensed_force_device_chai);
	hapticDevice->getTorque(_sensed_torque_device_chai);
	_sensed_force_device = convertChaiToEigenVector(-_sensed_force_device_chai);
	_sensed_torque_device = convertChaiToEigenVector(-_sensed_torque_device_chai);

 	// Transfer the desired force in the robot global frame
 	Vector3d _desired_force_robot;
 	Vector3d _desired_torque_robot;
	_desired_force_robot = _scaling_factor_trans * _Rotation_Matrix_DeviceToRobot.transpose() * _sensed_force_device;
	_desired_torque_robot = _scaling_factor_rot * _Rotation_Matrix_DeviceToRobot.transpose() * _sensed_torque_device;

 	// Compute the desired velocity through the set admittance
	_desired_trans_velocity_robot = _robot_trans_admittance *_desired_force_robot;
	_desired_rot_velocity_robot = _robot_rot_admittance *_desired_torque_robot;

	// Compute the force feedback in robot frame from the robot desired and current position (admittance-type scheme)
	Vector3d f_task_trans;
	Vector3d f_task_rot;
	Vector3d orientation_dev;
	cVector3d _force_chai;
	cVector3d _torque_chai;

	// Integrate the translational velocity error
	_integrated_trans_velocity_error += (_desired_trans_velocity_robot - _current_trans_velocity_robot) * _t_diff.count();
	// Evaluate the task force 
	f_task_trans = - _kp_robot_trans_velocity * (_desired_trans_velocity_robot - _current_trans_velocity_robot)- _ki_robot_trans_velocity * _integrated_trans_velocity_error;
	// Integrate the rotational velocity error
	_integrated_rot_velocity_error += (_desired_rot_velocity_robot - _current_rot_velocity_robot) * _t_diff.count();
	// Evaluate task torque
	f_task_rot = - _kp_robot_rot_velocity * (_desired_rot_velocity_robot - _current_rot_velocity_robot)- _ki_robot_rot_velocity * _integrated_rot_velocity_error;


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

    // Send set velocity to the robot 
	desired_trans_velocity_robot = _desired_trans_velocity_robot;
	desired_rot_velocity_robot = _desired_rot_velocity_robot;

}

void OpenLoopTeleop::computeHapticCommandsAdmittance3d(Eigen::Vector3d& desired_trans_velocity_robot)
{
	
	device_homed = false;

	// get time since last call for the I term
	_t_curr = std::chrono::high_resolution_clock::now();
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
		_integrated_trans_velocity_error.setZero();
	}
	_t_diff = _t_curr - _t_prev;

    // Update sensed force from the haptic device (if no force sensor the method considers the commanded forces)
	cVector3d _sensed_force_device_chai;
	hapticDevice->getForce(_sensed_force_device_chai);
	_sensed_force_device = convertChaiToEigenVector(-_sensed_force_device_chai);

	cout << "Force from chai" << _sensed_force_device << endl;


 	// Transfer the desired force in the robot global frame
 	Vector3d _desired_force_robot;
	_desired_force_robot = _scaling_factor_trans * _Rotation_Matrix_DeviceToRobot.transpose() * _sensed_force_device;

 	// Compute the desired velocity through the set admittance
	_desired_trans_velocity_robot = _robot_trans_admittance *_desired_force_robot;

	// Compute the force feedback in robot frame from the robot desired and current position (admittance-type scheme)
	Vector3d f_task_trans;
	Vector3d f_task_rot;
	Vector3d orientation_dev;
	cVector3d _force_chai;
	cVector3d _torque_chai;

	// Integrate the translational velocity error
	_integrated_trans_velocity_error += (_desired_trans_velocity_robot - _current_trans_velocity_robot) * _t_diff.count();
	// Evaluate the task force 
	f_task_trans = - _kp_robot_trans_velocity * (_desired_trans_velocity_robot - _current_trans_velocity_robot)- _ki_robot_trans_velocity * _integrated_trans_velocity_error;
	f_task_rot.setZero();

	// Apply reduction factors to force feedback
	f_task_trans = _reduction_factor_force_feedback * f_task_trans;
	//Transfer task force from robot to haptic device global frame
	_commanded_force_device = _Rotation_Matrix_DeviceToRobot * f_task_trans;
	_commanded_torque_device = _Rotation_Matrix_DeviceToRobot * f_task_rot;

	// Scaling of the force feedback
	Eigen::Matrix3d scaling_factor_trans;

		scaling_factor_trans << 1/_scaling_factor_trans, 0.0, 0.0,
						  0.0, 1/_scaling_factor_trans, 0.0, 
						  0.0, 0.0, 1/_scaling_factor_trans;

	_commanded_force_device = scaling_factor_trans * _commanded_force_device;

	// Saturate to Force and Torque limits of the haptic device
	if (_commanded_force_device.norm() >= device_info.m_maxLinearForce)
	{
		_commanded_force_device = device_info.m_maxLinearForce*_commanded_force_device/(_commanded_force_device.norm());
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

    // Send set velocity to the robot 
	desired_trans_velocity_robot = _desired_trans_velocity_robot;

}



///////////////////////////////////////////////////////////////////////////////////
// Impedance controllers with workspace extension algorithm
///////////////////////////////////////////////////////////////////////////////////
void OpenLoopTeleop::computeHapticCommandsWorkspaceExtension6d(Eigen::Vector3d& desired_position_robot,
							Eigen::Matrix3d& desired_rotation_robot)
{
	// get time since last call
	 _t_curr = std::chrono::high_resolution_clock::now();
	 if(_first_iteration)
	 {
	 	_t_prev = std::chrono::high_resolution_clock::now();
	 	_first_iteration = false;
	 	_desired_position_robot = _current_position_robot;
	 	_desired_rotation_robot = _current_rotation_robot;
	 	_center_position_robot_drift = _center_position_robot;
	 	_center_rotation_robot_drift = _center_rotation_robot;

	 	// Reinitialize maximum device velocity for the task 
	 	_max_rot_velocity_device=0.001;
		_max_trans_velocity_device=0.001;

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
	
	hapticDevice->getRotation(_current_rotation_device_chai);
	_current_rotation_device = convertChaiToEigenMatrix(_current_rotation_device_chai);	
	hapticDevice->getAngularVelocity(_current_rot_velocity_device_chai);
	_current_rot_velocity_device = convertChaiToEigenVector(_current_rot_velocity_device_chai);

	//Transfer device velocity to robot global frame
	_current_trans_velocity_device_RobFrame = _scaling_factor_trans * _Rotation_Matrix_DeviceToRobot.transpose() * _current_trans_velocity_device;
	_current_rot_velocity_device_RobFrame = _scaling_factor_rot * _Rotation_Matrix_DeviceToRobot.transpose() * _current_rot_velocity_device; 

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
		f_task_trans = _proxy_position_impedance*(_current_position_proxy - _desired_position_robot) - _proxy_position_damping * (_current_trans_velocity_device_RobFrame - _current_trans_velocity_proxy);

		// Compute the orientation error
		Sai2Model::orientationError(orientation_dev, _desired_rotation_robot, _current_rotation_proxy);
		// Evaluate task torque
		f_task_rot = _proxy_orientation_impedance*orientation_dev - _proxy_orientation_damping * (_current_rot_velocity_device_RobFrame - _current_rot_velocity_proxy);
		
		
	}
	else
	{
		// Read sensed task force 
		f_task_trans = _sensed_task_force.head(3);
		f_task_rot = _sensed_task_force.tail(3);																				//
	
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


	_device_force = _commanded_force_device;
	_device_torque = _commanded_torque_device;
	
	//// Evaluation of the drift force ////
	// Definition of the velocity gains from task feedback
	Matrix3d _Kv_translation = _drift_force_admissible_ratio*(_commanded_force_device.asDiagonal());
	Matrix3d _Kv_rotation = _drift_force_admissible_ratio*(_commanded_torque_device.asDiagonal());


	// Drift force computation
	_drift_force = _Kv_translation * _drift_trans_velocity;
	_drift_torque = _Kv_rotation * _drift_rot_velocity;
	//_drift_force = Lambda*_drift_force_0; //Drift force weigthed through the device inertia matrix

	// cout << "Fdrift \n" << _drift_force << endl;
	// cout << "Cdrift \n" << _drift_torque << endl;

	//// Desired cartesian force to apply to the haptic device ////
	_commanded_force_device += _drift_force;
	_commanded_torque_device += _drift_torque;

	if(!_send_haptic_feedback)
	{
		_commanded_force_device.setZero();
		_commanded_torque_device.setZero();
	}

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



void OpenLoopTeleop::computeHapticCommandsWorkspaceExtension3d(Eigen::Vector3d& desired_position_robot)
{
	// get time since last call
	 _t_curr = std::chrono::high_resolution_clock::now();
	 if(_first_iteration)
	 {
	 	_t_prev = std::chrono::high_resolution_clock::now();
	 	_first_iteration = false;
	 	_desired_position_robot = _current_position_robot;
	 	_desired_rotation_robot = _current_rotation_robot;
	 	_center_position_robot_drift = _center_position_robot;
	 	_center_rotation_robot_drift = _center_rotation_robot;

	 	_max_trans_velocity_device = 0.001;
	 }
	 _t_diff = _t_curr - _t_prev;

	device_homed = false;

 	cVector3d _current_position_device_chai;
 	cVector3d _current_trans_velocity_device_chai;
	// read haptic device position and velocity
	hapticDevice->getPosition(_current_position_device_chai);
	hapticDevice->getLinearVelocity(_current_trans_velocity_device_chai);
	_current_position_device = convertChaiToEigenVector(_current_position_device_chai);
	_current_trans_velocity_device = convertChaiToEigenVector(_current_trans_velocity_device_chai);

	// Update the maximum velocities for the task
	if (_current_trans_velocity_device.norm()>=_max_trans_velocity_device)	
	{
		_max_trans_velocity_device = _current_trans_velocity_device.norm();
	}

	//Transfer device velocity to robot global frame
	_current_trans_velocity_device_RobFrame = _scaling_factor_trans * _Rotation_Matrix_DeviceToRobot.transpose() * _current_trans_velocity_device;
	
	// Compute the force feedback in robot frame
	Vector3d f_task_trans;
	Vector3d f_task_rot;
	cVector3d _force_chai;
	cVector3d _torque_chai;
	
	if (_haptic_feedback_from_proxy)
	{
		// Evaluate the task force through stiffness proxy
		f_task_trans = _proxy_position_impedance*(_current_position_proxy - _desired_position_robot) - _proxy_position_damping * (_current_trans_velocity_device_RobFrame - _current_trans_velocity_proxy);
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

	//// Evaluation of the drift velocities ////
	//Compute the drift direction
	Vector3d relative_position_device;
	relative_position_device = _current_position_device -_home_position_device;
	Vector3d drift_unit_vector;
	drift_unit_vector = - relative_position_device/_device_workspace_radius_max;

	//_drift_trans_velocity = _current_trans_velocity_device.norm()*drift_unit_vector/_max_trans_velocity_device;
	_drift_trans_velocity = _drift_velocity_admissible_ratio*_current_trans_velocity_device.norm()*drift_unit_vector;

	//// Computation of the scaling factors ////
	_scaling_factor_trans = 1.0 + relative_position_device.norm()*(_task_workspace_radius_max/_device_workspace_radius_max-1.0)/_device_workspace_radius_max;

	// Scaling of the force feedback
	Eigen::Matrix3d scaling_factor_trans;
	scaling_factor_trans << 1/_scaling_factor_trans, 0.0, 0.0,
						  0.0, 1/_scaling_factor_trans, 0.0, 
						  0.0, 0.0, 1/_scaling_factor_trans;

	_commanded_force_device = scaling_factor_trans * _commanded_force_device;

	// cout << "Ks \n" << _scaling_factor_trans << endl;

	//// Evaluation of the drift force ////
	// Definition of the velocity gains from task feedback
	//Matrix3d _Kv_translation = _drift_force_admissible_ratio*(_commanded_force_device.asDiagonal());
	// Drift force computation
	//_drift_force = _Kv_translation * _drift_trans_velocity;
	_drift_force = _drift_force_admissible_ratio * _commanded_force_device.norm() * drift_unit_vector;

	// Compute total drift force to apply to the device
	_drift_force = 10.0*(_drift_trans_velocity-_current_trans_velocity_device)+_drift_force; //////////////////////////////////
	
	//cout << "Fdrift \n" << _drift_force << endl;

















	//// Desired cartesian force to apply to the haptic device ////
	_commanded_force_device += _drift_force;

	if(!_send_haptic_feedback)
	{
		_commanded_force_device.setZero();
	}

	//// Add virtual forces according to the device operational space limits ////
	Vector3d force_virtual = Vector3d::Zero();

	if (relative_position_device.norm() >= _device_workspace_radius_max)
	{
		force_virtual = -(0.8 * device_info.m_maxLinearStiffness * (relative_position_device.norm()-_device_workspace_radius_max)/(relative_position_device.norm()))*relative_position_device;
	}
	_commanded_force_device += force_virtual;

	if (_enable_plane_guidance_3D)
	{
		ComputePlaneGuidanceForce();
		_commanded_force_device += _guidance_force_plane;
	}

	if (_enable_line_guidance_3D)
	{
		ComputeLineGuidanceForce();
		_commanded_force_device += _guidance_force_line;
	}

	// Saturate to Force and Torque limits of the haptic device
	if (_commanded_force_device.norm() >= device_info.m_maxLinearForce)
	{
		_commanded_force_device = device_info.m_maxLinearForce*_commanded_force_device/(_commanded_force_device.norm());
	}

	// Send controllers force and torque to haptic device
	_force_chai = convertEigenToChaiVector(_commanded_force_device);
	_torque_chai = convertEigenToChaiVector(_commanded_torque_device);
    hapticDevice->setForceAndTorque(_force_chai,_torque_chai);

	//// Computation of the desired position for the controlled robot after drift of the device ////
	// Estimated drift velocity considering drift force 
	Vector3d _vel_drift_est_trans = _t_diff.count()*_drift_force;

	// Drift of the center of the task workspace
	_center_position_robot_drift -= _Rotation_Matrix_DeviceToRobot.transpose()*(_t_diff.count()*_scaling_factor_trans*_vel_drift_est_trans);

	cout << "Center robot WS \n" << _center_position_robot_drift << endl;

	//// Compute position of the controlled robot after drift ////
	 // Compute the set position from the haptic device
	_desired_position_robot = _scaling_factor_trans*relative_position_device;

	//Transfer set position and orientation from device to robot global frame
	_desired_position_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_position_robot;

	// Adjust set position to the center of the task Workspace
	_desired_position_robot = _desired_position_robot + _center_position_robot_drift;

	// Send set position orientation of the robot
	desired_position_robot = _desired_position_robot;

	// update previous time
	 _t_prev = _t_curr;


}









///////////////////////////////////////////////////////////////////////////////////
// Haptic guidance related methods 
///////////////////////////////////////////////////////////////////////////////////
void OpenLoopTeleop::ComputePlaneGuidanceForce()
{
	// vector from the plane's origin to the current position
	Eigen::Vector3d plane_to_point_vec;
	plane_to_point_vec = _current_position_device - _plane_origin_point;

	// calculate the normal distance from the plane to the current position
	double distance_to_plane;
	distance_to_plane = plane_to_point_vec.dot(_plane_normal_vec);

	// current position projected onto the plane
	Eigen::Vector3d projected_current_position;
	projected_current_position = _current_position_device - distance_to_plane * _plane_normal_vec;

	// projected device velocity on the normal vector
	Eigen::Vector3d projected_current_velocity;
	projected_current_velocity = _current_trans_velocity_device.dot(_plane_normal_vec)*_plane_normal_vec;

	// calculate the force feedback
	_guidance_force_plane = - _guidance_stiffness * device_info.m_maxLinearStiffness * (_current_position_device - projected_current_position) - _guidance_damping * projected_current_velocity;
}

void OpenLoopTeleop::ComputeLineGuidanceForce()
{
	Eigen::Vector3d line_to_point_vec;
	double distance_along_line;
	Eigen::Vector3d closest_point_vec;
	Eigen::Vector3d projected_current_position;
	Eigen::Vector3d line_normal_vec;
	
	// vector from first point on line to current position
	line_to_point_vec = _current_position_device - _line_first_point;

	// distance from the first point to the current position projected onto the line
	distance_along_line = line_to_point_vec.dot(_guidance_line_vec);

	// vector to the projected point onto the line from the first point
	closest_point_vec = distance_along_line * _guidance_line_vec;

	// projected position on line in world frame
	projected_current_position = _line_first_point + closest_point_vec;

	// Normal vector to the line from current position
	line_normal_vec = _current_position_device - projected_current_position;
	line_normal_vec = line_normal_vec/line_normal_vec.norm();

	// projected device velocity on the normal vector
	Eigen::Vector3d projected_current_velocity;
	projected_current_velocity = _current_trans_velocity_device.dot(line_normal_vec)*line_normal_vec;

	// compute the force 
	_guidance_force_line = - (_guidance_stiffness) * device_info.m_maxLinearStiffness * (_current_position_device - projected_current_position) - _guidance_damping * projected_current_velocity;
}


///////////////////////////////////////////////////////////////////////////////////
// Updating methods for haptic feedback computation
///////////////////////////////////////////////////////////////////////////////////
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

void OpenLoopTeleop::updateVirtualProxyPositionVelocity(const Eigen::Vector3d current_position_proxy,
								const Eigen::Vector3d current_trans_velocity_proxy,
								const Eigen::Matrix3d current_rotation_proxy,
								const Eigen::Vector3d current_rot_velocity_proxy)
{
	_current_position_proxy = current_position_proxy;
	_current_rotation_proxy = current_rotation_proxy;
	_current_trans_velocity_proxy = current_trans_velocity_proxy;
	_current_rot_velocity_proxy = current_rot_velocity_proxy;
}

void OpenLoopTeleop::updateSelectionMatrices(const Eigen::Matrix3d sigma_position, const Eigen::Matrix3d sigma_orientation,
								const Eigen::Matrix3d sigma_force, const Eigen::Matrix3d sigma_moment)
{
	_sigma_position = _Rotation_Matrix_DeviceToRobot * sigma_position * _Rotation_Matrix_DeviceToRobot.transpose();
	_sigma_orientation = _Rotation_Matrix_DeviceToRobot * sigma_orientation * _Rotation_Matrix_DeviceToRobot.transpose();
	_sigma_force = _Rotation_Matrix_DeviceToRobot * sigma_force * _Rotation_Matrix_DeviceToRobot.transpose();
	_sigma_moment = _Rotation_Matrix_DeviceToRobot * sigma_moment * _Rotation_Matrix_DeviceToRobot.transpose();
}



///////////////////////////////////////////////////////////////////////////////////
// Haptic device specific methods
///////////////////////////////////////////////////////////////////////////////////
void OpenLoopTeleop::initializeSigmaDevice()
{
	if(device_info.m_modelName == "sigma.7")
	{
		cDeltaDevice* tmp_device = static_cast<cDeltaDevice*>(hapticDevice.get());
		tmp_device->enableForces(true);
	}	
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



///////////////////////////////////////////////////////////////////////////////////
// Parameter setting methods
///////////////////////////////////////////////////////////////////////////////////

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

void OpenLoopTeleop::setForceFeedbackCtrlGains (const double kp_robot_trans_velocity, const double ki_robot_trans_velocity,
										const double kp_robot_rot_velocity, const double ki_robot_rot_velocity,
										const double robot_trans_admittance,
										const double robot_rot_admittance,
										const Matrix3d reduction_factor_force_feedback,
										const Matrix3d reduction_factor_torque_feedback)
{
	_kp_robot_trans_velocity = kp_robot_trans_velocity;
	_kp_robot_rot_velocity = kp_robot_rot_velocity;
	_ki_robot_trans_velocity = ki_robot_trans_velocity;
	_ki_robot_rot_velocity = ki_robot_rot_velocity;
	_robot_trans_admittance = robot_trans_admittance;
	_robot_rot_admittance = robot_rot_admittance;
	_reduction_factor_force_feedback = reduction_factor_force_feedback;
	_reduction_factor_torque_feedback = reduction_factor_torque_feedback;
}


void OpenLoopTeleop::setVirtualProxyGains (const double proxy_position_impedance, const double proxy_position_damping,
									const double proxy_orientation_impedance, const double proxy_orientation_damping)
{
	_proxy_position_impedance = proxy_position_impedance;
	_proxy_position_damping = proxy_position_damping;
	_proxy_orientation_impedance = proxy_orientation_impedance;
	_proxy_orientation_damping = proxy_orientation_damping;
}

void OpenLoopTeleop::setVirtualGuidanceGains (const double force_guidance_position_impedance, const double force_guidance_position_damping,
									const double force_guidance_orientation_impedance, const double force_guidance_orientation_damping)
{
	_force_guidance_position_impedance = force_guidance_position_impedance;
	_force_guidance_position_damping = force_guidance_position_damping;
	_force_guidance_orientation_impedance = force_guidance_orientation_impedance;
	_force_guidance_orientation_damping = force_guidance_orientation_damping;
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

void OpenLoopTeleop::setWorkspaceLimits(double device_workspace_radius_limit, double device_workspace_angle_limit)
{
	_device_workspace_radius_limit = device_workspace_radius_limit;
	_device_workspace_angle_limit = device_workspace_angle_limit;
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


void OpenLoopTeleop::setNoticeableDiff(double drift_force_admissible_ratio, double drift_velocity_admissible_ratio)

{
	_drift_force_admissible_ratio = drift_force_admissible_ratio;
	_drift_velocity_admissible_ratio = drift_velocity_admissible_ratio;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Haptic guidance related settings methods ////

void OpenLoopTeleop::setHapticGuidanceGains(const double guidance_stiffness, const double guidance_damping)
{
	_guidance_stiffness = guidance_stiffness;
	_guidance_damping = guidance_damping;
}

void OpenLoopTeleop::setPlane(const Eigen::Vector3d plane_origin_point, const Eigen::Vector3d plane_normal_vec)
{
	_plane_origin_point = plane_origin_point;
	_plane_normal_vec = plane_normal_vec / plane_normal_vec.norm();
}

void OpenLoopTeleop::setLine(const Eigen::Vector3d line_first_point, const Eigen::Vector3d line_second_point)
{
	//set vector from one point to the other
	_line_first_point = line_first_point;
	_line_second_point = line_second_point;

	_guidance_line_vec = line_second_point - line_first_point;
	_guidance_line_vec = _guidance_line_vec / _guidance_line_vec.norm();
}



} /* namespace Sai2Primitives */

