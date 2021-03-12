/*
 * ImpedanceControl.cpp
 *
 *    This controller implements the classic impedance bilateral scheme for haptic teleoperation.
 * 		The desired force feedback is computed for the haptic device (force feedback) and the desired position and rotation
 *    are evaluated for the robot (desired_position_robot & desired_rotation_robot).
 * 		The impedance-type controller can additionally include plane/line/orientation guidances and virtual workspace limits.
 *
 *     Author: Margot Vulliez
 */

#include "ImpedanceControl.h"


 namespace Sai2Primitives
 {

////////////////////////////////////////////////////////////////////////////////////////////////////
//// Constructor, Destructor and Initialization of the impedance-type controller
////////////////////////////////////////////////////////////////////////////////////////////////////
// This constructor creates the haptic impedance controller for a simple bilateral teleoperation scheme.
 ImpedanceControl::ImpedanceControl(const Vector3d center_position_robot,
                 const Matrix3d center_rotation_robot,
                 const Vector3d max_device_force,
                 const Vector2d max_device_stiffness,
                 const Vector2d max_device_damping,
                 const Matrix3d Rotation_Matrix_DeviceToRobot)
 {
   //Send zero force feedback to the haptic device
   _commanded_force_device.setZero();
   _commanded_torque_device.setZero();
   _commanded_gripper_force_device = 0.0;
   sendFeedback(true);

   //Initialize sensed force to zero
   updateRobotSensedForce();

   //Initialize device position to defaut center
   updateDevicePosition();

   //Initialize device velocities to zero
   updateDeviceVelocity();

   //Set default device center position
   setDeviceCenter();

   // Initialize robot task workspace center
   setRobotCenter(center_position_robot, center_rotation_robot);

   //Initialize gripper to be closed
   _desired_gripper_position_robot = 0.0;

   //Initialize transformation matrix from device to robot frames
   setDeviceRobotRotation(Rotation_Matrix_DeviceToRobot);

   // Initialize scaling factors to 1
   setScalingFactors();

   // Initialiaze force feedback reduction factor  to 1
   setReductionFactorForceFeedback ();

   // Set device maximum force, stiffness, and damping
   setDeviceMaxSpecification(max_device_force, max_device_stiffness, max_device_damping);

   // Turn off additional controller fuctions
   setFilterOn(false);
   setWorkspaceLimits(false);

   // Create Butterworth filters for sensed forces
 	_force_filter = new ButterworthFilter(3);
 	_moment_filter = new ButterworthFilter(3);
  _gripper_force_filter = new ButterworthFilter(3);

  // Gripper is not initialized to be used as switch
	gripper_init = false;

  //Initialize haptic guidance parameters
  setHapticGuidanceGains(0.6 * _max_linear_stiffness_device, 0.8 * _max_linear_damping_device);
  setGuidancePlane(false);
  setGuidanceLine(false);
}

// This destructor deletes the pointers and stop the haptic controller.
 ImpedanceControl::~ImpedanceControl()
 {
   //Send zero force feedback to the haptic device
   _commanded_force_device.setZero();
   _commanded_torque_device.setZero();
   _commanded_gripper_force_device = 0.0;
   // Stop haptic feedback
   sendFeedback(false);

   //Delete filters
   delete _force_filter;
   _force_filter = NULL;
   delete _moment_filter;
   _moment_filter = NULL;
   delete _gripper_force_filter;
   _gripper_force_filter = NULL;
 }

 ////////////////////////////////////////////////////////////////////////////////////////////////////
 // Impedance-type bilateral controller - Core Methods (Data updates and commands' computation)
 ////////////////////////////////////////////////////////////////////////////////////////////////////
// Compute haptic device and robot commands and integrate additional guidances or workspace limits
 void ImpedanceControl::computeCommands(Vector3d& desired_position_robot,
                        Matrix3d& desired_rotation_robot,
                        Vector3d& commanded_force_device,
                        Vector3d& commanded_torque_device)
  {
    // Position of the device with respect to home position
    Vector3d relative_position_device;
    relative_position_device = _current_position_device-_home_position_device;
    // Rotation of the device with respect to home orientation
    Matrix3d relative_rotation_device = _current_rotation_device * _home_rotation_device.transpose();
    AngleAxisd relative_orientation_angle_device = AngleAxisd(relative_rotation_device);

    // Compute the force feedback in robot frame
    // Apply reduction factors to force feedback
    _commanded_force_device = _reduction_factor_force_feedback * _sensed_task_force;
    _commanded_torque_device = _reduction_factor_torque_feedback * _sensed_task_torque;

    // Transfer task force from robot to haptic device global frame
    _commanded_force_device = _Rotation_Matrix_DeviceToRobot * _commanded_force_device;
    _commanded_torque_device = _Rotation_Matrix_DeviceToRobot * _commanded_torque_device;

    // Scaling of the force feedback
    _commanded_force_device = _commanded_force_device / _scaling_factor_trans;
    _commanded_torque_device = _commanded_torque_device / _scaling_factor_rot;

    // Apply haptic guidances if activated
    if (_enable_plane_guidance)
    {
      ComputePlaneGuidanceForce();
      _commanded_force_device += _guidance_force_plane - _commanded_force_device.dot(_plane_normal_vec) * _plane_normal_vec;
    }
    if (_enable_line_guidance)
    {
      ComputeLineGuidanceForce();
      _commanded_force_device = _guidance_force_line + _commanded_force_device.dot(_guidance_line_vec) * _guidance_line_vec;
    }

    // If activated, add virtual forces according to the device operational space limits
    if(_add_workspace_virtual_limit)
    {
      Vector3d force_virtual = Vector3d::Zero();
      Vector3d torque_virtual = Vector3d::Zero();

      if (relative_position_device.norm() >= _device_workspace_radius_limit)
      {
        force_virtual = -(0.8 * _max_linear_stiffness_device * (relative_position_device.norm()-_device_workspace_radius_limit)/(relative_position_device.norm()))*relative_position_device;
      }
      _commanded_force_device += force_virtual;

      if (relative_orientation_angle_device.angle() >= _device_workspace_angle_limit)
      {
        torque_virtual = -(0.8 * _max_angular_stiffness_device *(relative_orientation_angle_device.angle()-_device_workspace_angle_limit))*relative_orientation_angle_device.axis();
      }
      _commanded_torque_device += torque_virtual;
    }

    // Saturate to Force and Torque limits of the haptic device
    if (_commanded_force_device.norm() > _max_force_device)
    {
      _commanded_force_device = _max_force_device*_commanded_force_device/(_commanded_force_device.norm());
    }
    if (_commanded_torque_device.norm() > _max_torque_device)
    {
      _commanded_torque_device = _max_torque_device*_commanded_torque_device/(_commanded_torque_device.norm());
    }

    // Reset to zero if haptic feedback disabled
    if(!_send_haptic_feedback)
    {
      _commanded_force_device.setZero();
      _commanded_torque_device.setZero();
    }

    // Compute the new desired position from the haptic device
    _desired_position_robot = _scaling_factor_trans*relative_position_device;

    // Compute set orientation from the haptic device
    Eigen::AngleAxisd desired_rotation_robot_aa = Eigen::AngleAxisd(_scaling_factor_rot*relative_orientation_angle_device.angle(),relative_orientation_angle_device.axis());
    _desired_rotation_robot = desired_rotation_robot_aa.toRotationMatrix();

    //Transfer set position and orientation from device to robot global frame
    _desired_position_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_position_robot + _center_position_robot;
    _desired_rotation_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_rotation_robot * _Rotation_Matrix_DeviceToRobot * _center_rotation_robot;

      // Send desired position and rotation to the robot
    desired_position_robot = _desired_position_robot;
    desired_rotation_robot = _desired_rotation_robot;
    // Send desired force/torque feedback to the device
    commanded_force_device = _commanded_force_device;
    commanded_torque_device = _commanded_torque_device;
  }

// Compute haptic device and robot gripper commands
 void ImpedanceControl::computeGripperCommands(double& desired_gripper_position_robot, double& commanded_gripper_force_device)
 {
   // Compute gripper force feedback in robot frame
   // Apply reduction factor
   _commanded_gripper_force_device = _reduction_factor_gripper_force * _sensed_task_gripper_force;

   // Scaling of the force feedback
   _commanded_gripper_force_device = _commanded_gripper_force_device / _scaling_factor_gripper;

   // Saturate to max gripper force of the haptic device
   if (_commanded_gripper_force_device > _max_gripper_force_device)
   {
     _commanded_gripper_force_device = _max_gripper_force_device;
   }
   else if (_commanded_gripper_force_device < -_max_gripper_force_device)
   {
     _commanded_gripper_force_device = -_max_gripper_force_device;
   }

   // Reset to zero if haptic feedback disabled
   if(!_send_haptic_feedback)
   {
     _commanded_gripper_force_device = 0.0;
   }

   // Compute the new desired gripper position from the haptic device
   _desired_gripper_position_robot = _scaling_factor_gripper*_current_position_gripper_device;

     // Send desired gripper position to the robot
   desired_gripper_position_robot = _desired_gripper_position_robot;
   // Send desired gripper haptic feedback to the device
   commanded_gripper_force_device = _commanded_gripper_force_device;
 }

// Send desired haptic feedback to the device
 void ImpedanceControl::sendFeedback(bool feedback_OnOff)
 {
   _send_haptic_feedback = feedback_OnOff;
   if (!_send_haptic_feedback)
   {
     _commanded_force_device.setZero();
     _commanded_torque_device.setZero();
     _commanded_gripper_force_device = 0.0;
   }
 }

// Update robot sensed task force, torque and gripper force, in robot frame.
 void ImpedanceControl::updateRobotSensedForce(const Vector3d sensed_task_force, const Vector3d sensed_task_torque,
                        const double sensed_task_gripper_force)
{
  if (_filter_on)
  {
    _sensed_task_force = _force_filter->update(sensed_task_force);
    _sensed_task_torque = _moment_filter->update(sensed_task_torque);
    Vector3d task_gripper_force;
    Vector3d filtered_task_gripper_force;
    task_gripper_force << sensed_task_gripper_force, 0.0, 0.0;
    filtered_task_gripper_force = _gripper_force_filter->update(task_gripper_force);
    _sensed_task_gripper_force = filtered_task_gripper_force[0];
  }
  else
  {
    _sensed_task_force = sensed_task_force;
    _sensed_task_torque = sensed_task_torque;
    _sensed_task_gripper_force = sensed_task_gripper_force;
  }
}

//Update haptic device position data (position, rotation, and gripper position), in haptic frame.
void ImpedanceControl::updateDevicePosition(const Vector3d current_position_device, const Matrix3d current_rotation_device,
                          const double current_position_gripper_device)
{
  _current_position_device = current_position_device;
  _current_rotation_device = current_rotation_device;
  _current_position_gripper_device = current_position_gripper_device;
}

//Update haptic device velocity data in haptic frame.
void ImpedanceControl::updateDeviceVelocity(const Vector3d current_trans_velocity_device, const Vector3d current_rot_velocity_device,
                          const double current_gripper_velocity_device)
{
    _current_trans_velocity_device = current_trans_velocity_device;
    _current_rot_velocity_device = current_rot_velocity_device;
    _current_gripper_velocity_device = current_gripper_velocity_device;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Impedance-type bilateral controller - Specification Methods (To change controller factors, or device/robot task origin)
////////////////////////////////////////////////////////////////////////////////////////////////////
// Set the center of the device workspace in device frame
 void ImpedanceControl::setDeviceCenter(const Vector3d center_position_device,
                     const Matrix3d center_rotation_device)
 {
   _home_position_device = center_position_device;
   _home_rotation_device = center_rotation_device;
 }

//Set the center of the robot task workspace in robot frame
 void ImpedanceControl::setRobotCenter(const Vector3d center_position_robot,
                     const Matrix3d center_rotation_robot)
 {
   _center_position_robot = center_position_robot;
   _center_rotation_robot = center_rotation_robot;
   //Initialize the desired robot to center
   _desired_position_robot = _center_position_robot;
   _desired_rotation_robot = _center_rotation_robot;
 }

 // Set the rotation matrix from haptic device operational space to robot operational space
 void ImpedanceControl::setDeviceRobotRotation(const Matrix3d Rotation_Matrix_DeviceToRobot)
 {
   _Rotation_Matrix_DeviceToRobot = Rotation_Matrix_DeviceToRobot;
 }

 // Set haptic device specifications (max stiffness, max damping, and max force/torque/gripper force)
  void ImpedanceControl::setDeviceMaxSpecification (const Vector3d max_device_force,
                                                    const Vector2d max_device_stiffness,
                                                    const Vector2d max_device_damping)
 {
   // copy max stiffness values
   _max_linear_stiffness_device = max_device_stiffness[0];
   _max_angular_stiffness_device = max_device_stiffness[1];
   // copy max damping values
   _max_linear_damping_device = max_device_damping[0];
   _max_angular_damping_device = max_device_damping[1];
   // copy max force values
   _max_force_device = max_device_force[0];
   _max_torque_device = max_device_force[1];
   _max_gripper_force_device = max_device_force[2];
 }

 // Set the scaling factors from the device workspace to the robot task workspace
 void ImpedanceControl::setScalingFactors(const double scaling_factor_trans, const double scaling_factor_rot, const double scaling_factor_gripper)
 {
   _scaling_factor_trans = scaling_factor_trans;
   _scaling_factor_rot = scaling_factor_rot;
   _scaling_factor_gripper = scaling_factor_gripper;

 }

 // Set an additional reduction factors between the sensed task force and the haptic rendered force
 void ImpedanceControl::setReductionFactorForceFeedback (const double reduction_factor_force_feedback, const double reduction_factor_torque_feedback,
                                       const double reduction_factor_gripper_force)
 {
   _reduction_factor_force_feedback = reduction_factor_force_feedback;
   _reduction_factor_torque_feedback = reduction_factor_torque_feedback;
   _reduction_factor_gripper_force = reduction_factor_gripper_force;
 }

 // Set stiffness/damping for haptic guidance
 void ImpedanceControl::setHapticGuidanceGains(const double guidance_stiffness, const double guidance_damping)
 {
   _guidance_stiffness = guidance_stiffness;
   _guidance_damping = guidance_damping;
 }

 // Set a virtual plane as haptic guidance
 void ImpedanceControl::setGuidancePlane(bool planeGuidance_OnOff, const Vector3d plane_origin_point, const Vector3d plane_normal_vec)
 {
   _enable_plane_guidance = planeGuidance_OnOff;
   // reset guidance force to zero
  _guidance_force_plane.setZero();
  // copy guidance plane parameters
   if(_enable_plane_guidance)
   {
     _plane_origin_point = plane_origin_point;
     _plane_normal_vec = plane_normal_vec / plane_normal_vec.norm();
   }
 }

 // Set a virtual line as haptic guidance
 void ImpedanceControl::setGuidanceLine(bool lineGuidance_OnOff, const Vector3d line_first_point, const Vector3d line_second_point)
 {
   _enable_line_guidance = lineGuidance_OnOff;
   // Reset guidance force to zero
   _guidance_force_line.setZero();
   // copy line parameters
  if (_enable_line_guidance)
  {
     _line_first_point = line_first_point;
     _line_second_point = line_second_point;
     _guidance_line_vec = line_second_point - line_first_point;
     _guidance_line_vec = _guidance_line_vec / _guidance_line_vec.norm();
  }
 }

////////////////////////////////////////////////////////////////////////////////////////////////////
// Impedance-type bilateral controller - Additional Methods
////////////////////////////////////////////////////////////////////////////////////////////////////
// Set a filter on robot force sensor data at the given normalized cut-off frequencies (between 0 and 0.5)
void ImpedanceControl::setFilterOn(bool filter_OnOff, const double cutOff_frequency_force,
                   const double cutOff_frequency_moment, const double cutOff_frequency_gripper_force)
  {
      _filter_on = filter_OnOff;
      if (_filter_on)
      {
          _cutOff_frequency_force = cutOff_frequency_force;
          _cutOff_frequency_moment = cutOff_frequency_moment;
          _cutOff_frequency_gripper_force = cutOff_frequency_gripper_force;
          _force_filter->setCutoffFrequency(_cutOff_frequency_force);
          _moment_filter->setCutoffFrequency(_cutOff_frequency_moment);
          _gripper_force_filter->setCutoffFrequency(_cutOff_frequency_gripper_force);
      }
  }

 //Add virtual limits to render the device workspace boundary through the force feedback
 void ImpedanceControl::setWorkspaceLimits(bool virtualWS_OnOff, double device_workspace_radius_limit,
                                          double device_workspace_angle_limit)
 {
     _add_workspace_virtual_limit = virtualWS_OnOff;
     if (_add_workspace_virtual_limit)
     {
     _device_workspace_radius_limit = device_workspace_radius_limit;
     _device_workspace_angle_limit = device_workspace_angle_limit;
    }
 }

 // A method to use the gripper like a switch
 void ImpedanceControl::useGripperAsSwitch(bool& gripper_state, double& commanded_gripper_force_device)
 {
   	double gripper_start_angle = 10*M_PI/180.0;
   	double gripper_switch_angle = 5*M_PI/180.0;
    double gripper_force_click = 3.0;
    double gripper_force_switched = 2.0;

    //Update damping force term from gripper velocity
  	double damping_force = -0.05 * _current_gripper_velocity_device;

    _desired_gripper_position_robot = 0.0; // Close robot gripper

   	// Initialization of the gripper at first call of the method
   	if (!gripper_init)
   	{
   		gripper_state = false;
   		if(_current_position_gripper_device <= gripper_switch_angle)
   		{
   			_commanded_gripper_force_device = gripper_force_switched + damping_force;
   		}
   		else
   		{
   			_commanded_gripper_force_device = 0.0;
   			gripper_init = true;
   		}
   	}
   	else
   	{
   		// case 0: outside of switch, zero force
   	    if (_current_position_gripper_device > gripper_start_angle)
   	    {
   	        _commanded_gripper_force_device = 0.0;
   	        gripper_state = false;
   	    }
   	    // case 1: switch is being engaged. (Force is rising until "click")
   	    else if ((_current_position_gripper_device <= gripper_start_angle) &&
   	             (_current_position_gripper_device > gripper_switch_angle))
   	    {
   	        double switching_force = (gripper_start_angle - _current_position_gripper_device) * ((gripper_force_click) / (gripper_start_angle - gripper_switch_angle));
   	        _commanded_gripper_force_device = switching_force + damping_force;
   	        gripper_state = false;
   	    }
   	    // case 2: switch has been engaged. (Force is constant)
   	    else if (_current_position_gripper_device <= gripper_switch_angle)
   	    {
   	        _commanded_gripper_force_device = gripper_force_switched + damping_force;
   	        gripper_state = true;
   	    }
   	}
    commanded_gripper_force_device = _commanded_gripper_force_device;
 }

// Compute guidance force to virtual plane fixture
 void ImpedanceControl::ComputePlaneGuidanceForce()
 {
   	// vector from the plane's origin to the current position
   	Vector3d plane_to_point_vec;
   	plane_to_point_vec = _current_position_device - _plane_origin_point;

   	// calculate the normal distance from the plane to the current position
   	double distance_to_plane;
   	distance_to_plane = plane_to_point_vec.dot(_plane_normal_vec);

   	// current position projected onto the plane
   	Vector3d projected_current_position;
   	projected_current_position = _current_position_device - distance_to_plane * _plane_normal_vec;

   	// projected device velocity on the normal vector
   	Vector3d projected_current_velocity;
   	projected_current_velocity = _current_trans_velocity_device.dot(_plane_normal_vec)*_plane_normal_vec;

   	// calculate the force feedback
   	_guidance_force_plane = - _guidance_stiffness * (_current_position_device - projected_current_position) - _guidance_damping * projected_current_velocity;
 }

 // Compute guidance force to virtual line fixture
 void ImpedanceControl::ComputeLineGuidanceForce()
 {
   	Vector3d line_to_point_vec;
   	double distance_along_line;
   	Vector3d closest_point_vec;
   	Vector3d projected_current_position;
   	Vector3d line_normal_vec;

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
   	Vector3d projected_current_velocity;
   	projected_current_velocity = _current_trans_velocity_device.dot(line_normal_vec)*line_normal_vec;

   	// compute the force
   	_guidance_force_line = - _guidance_stiffness * (_current_position_device - projected_current_position) - _guidance_damping * projected_current_velocity;
 }


} /* namespace Sai2Primitives */
