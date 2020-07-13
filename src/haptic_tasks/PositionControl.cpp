/*
 * PositionControl.h
 *
 *    This controller implements a simple operational space position controller for the haptic device.
 * 		The command force is computed for the haptic device with respect to the desired position through a PD gain.
 *
 *     Author: Margot Vulliez
 */

 #include "PositionControl.h"

 namespace Sai2Primitives
 {

////////////////////////////////////////////////////////////////////////////////////////////////////
//// Constructor, Destructor and Initialization of the impedance-type controller
////////////////////////////////////////////////////////////////////////////////////////////////////
//This constructor creates the haptic position controller in operational sapce.
PositionControl::PositionControl(const Vector3d max_device_force,
                                 const Vector2d max_device_stiffness,
                                 const Vector2d max_device_damping)
{
    //Send zero force feedback to the haptic device
    _commanded_force_device.setZero();
    _commanded_torque_device.setZero();
    _commanded_gripper_force_device = 0.0;

    //Initialize device position to defaut center
    updateDevicePosition();

    //Initialize device velocities to zero
    updateDeviceVelocity();

    //Set default device center position
    setDeviceCenter();

    // Set desired haptic position to device default center
    setDesiredPosition();

    // Set device maximum force, stiffness, and damping
    setDeviceMaxSpecification(max_device_force, max_device_stiffness, max_device_damping);

    // Set standard gains to position controller    TODO tune these gains !
    setPosCtrlGains(0.14 * _max_linear_stiffness_device, 0.6 * _max_linear_damping_device,
                    0.5 * _max_angular_stiffness_device, 0.2 * _max_angular_damping_device,
                    30.0, 0.3);

    // Turn off additional controller fuctions
    setWorkspaceLimits(false);

   // Gripper is not initialized to be used as switch
   gripper_init = false;
   // Device did not reach desired position
   isInPos = false;
}

//This destructor deletes the pointers and stop the position controller.
PositionControl::~PositionControl()
{
  //Send zero force feedback to the haptic device
  _commanded_force_device.setZero();
  _commanded_torque_device.setZero();
  _commanded_gripper_force_device = 0.0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Haptic position controller - Core Methods (Data updates and commands' computation)
////////////////////////////////////////////////////////////////////////////////////////////////////
//Computes the commands of the haptic device to follow the desired position
void PositionControl::computeCommands(Vector3d& commanded_force_device,
                                      Vector3d& commanded_torque_device)
{
  // Desired position with respect to workspace center
  Vector3d relative_desired_position;
  relative_desired_position = _desired_position_device - _home_position_device;
  // Desired rotation with respect to workspace center
  Matrix3d relative_desired_rotation = _desired_rotation_device * _home_rotation_device.transpose();
  AngleAxisd relative_desired_orientation_angle = AngleAxisd(relative_desired_rotation);

  // Saturate position commands if the workspace limits are active
  if(_add_workspace_virtual_limit)
  {
    if (relative_desired_position.norm() > _device_workspace_radius_limit)
    {
      _desired_position_device = _home_position_device + _device_workspace_radius_limit*relative_desired_position/(relative_desired_position.norm());
    }
    if (relative_desired_orientation_angle.angle() > _device_workspace_angle_limit)
    {
      relative_desired_orientation_angle.angle() = _device_workspace_angle_limit;
      _desired_rotation_device = relative_desired_orientation_angle.toRotationMatrix() * _home_rotation_device;
    }
  }

  Matrix3d relative_error_rotation = _desired_rotation_device * _current_rotation_device.transpose();
  AngleAxisd relative_error_orientation_angle = AngleAxisd(relative_error_rotation);

  // Compute position error
  Vector3d position_error;
  position_error = _current_position_device -_desired_position_device;
  // Compute the orientation error
  Vector3d orientation_error;
  Sai2Model::orientationError(orientation_error, _desired_rotation_device, _current_rotation_device);

  // Evaluate position controller force
  _commanded_force_device = -_kp_position_ctrl_device * position_error - _kv_position_ctrl_device * _current_trans_velocity_device;
  // Evaluate orientation controller force
  _commanded_torque_device = -_kp_orientation_ctrl_device * orientation_error - _kv_orientation_ctrl_device * _current_rot_velocity_device;

  // Saturate to Force and Torque limits of the haptic device
  if (_commanded_force_device.norm() > _max_force_device)
  {
    _commanded_force_device = _max_force_device*_commanded_force_device/(_commanded_force_device.norm());
  }
  if (_commanded_torque_device.norm() > _max_torque_device)
  {
    _commanded_torque_device = _max_torque_device*_commanded_torque_device/(_commanded_torque_device.norm());
  }

  // Send desired force/torque feedback to the device
  commanded_force_device = _commanded_force_device;
  commanded_torque_device = _commanded_torque_device;

  // Check position error
  if(position_error.norm()<0.002)
  {
    isInPos = true;
  }
  else
  {
    isInPos = false;
  }
}

//Computes the commands of the haptic device to follow the desired position
void PositionControl::computeGripperCommands(double& commanded_gripper_force_device)
{
    // Compute gripper position error
    double position_error;
    position_error = _current_position_gripper_device - _desired_gripper_position_device;

    // Evaluate position controller gripper force
    _commanded_gripper_force_device = -_kp_position_ctrl_gripper * position_error - _kv_position_ctrl_gripper * _current_gripper_velocity_device;

    // Saturate to max gripper force of the haptic device
    if (_commanded_gripper_force_device > _max_gripper_force_device)
    {
      _commanded_gripper_force_device = _max_gripper_force_device;
    }
    else if (_commanded_gripper_force_device < -_max_gripper_force_device)
    {
      _commanded_gripper_force_device = -_max_gripper_force_device;
    }

    std::cout << "gripper pos error" << position_error  << std::endl;
    std::cout << "gripper force" <<_commanded_gripper_force_device << std::endl;
    std::cout << "max gripper force" << _max_gripper_force_device  << std::endl;

    // Send desired gripper haptic feedback to the device
    commanded_gripper_force_device = _commanded_gripper_force_device;
}

//Update Haptic device position data (position, rotation, and gripper position), in haptic frame.
 void PositionControl::updateDevicePosition(const Vector3d current_position_device, const Matrix3d current_rotation_device,
                           const double current_position_gripper_device)
{
    _current_position_device = current_position_device;
    _current_rotation_device = current_rotation_device;
    _current_position_gripper_device = current_position_gripper_device;
}

//Update Haptic device velocity data (translational and rotational device velocities and gripper velocity), in haptic frame.
void PositionControl::updateDeviceVelocity(const Vector3d current_trans_velocity_device, const Vector3d current_rot_velocity_device,
                            const double current_gripper_velocity_device)
{
    _current_trans_velocity_device = current_trans_velocity_device;
    _current_rot_velocity_device = current_rot_velocity_device;
    _current_gripper_velocity_device = current_gripper_velocity_device;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Haptic position controller - Specification Methods (To change set desired values and controller factors)
////////////////////////////////////////////////////////////////////////////////////////////////////
// Set the desired position for the haptic device in operational space
void PositionControl::setDesiredPosition(const Vector3d desired_position_device,
                    const Matrix3d desired_rotation_device,
                    const double desired_gripper_position_device)
{
    _desired_position_device = desired_position_device;
    _desired_rotation_device = desired_rotation_device;
    _desired_gripper_position_device = desired_gripper_position_device;
}

//Set the center of the device workspace in device frame, from which workspace boundaries are set
void PositionControl::setDeviceCenter(const Vector3d center_position_device,
                    const Matrix3d center_rotation_device)
{
  _home_position_device = center_position_device;
  _home_rotation_device = center_rotation_device;
}

//Set haptic device specifications (max stiffness, max damping, and max force/torque/gripper force)
void PositionControl::setDeviceMaxSpecification (const Vector3d max_device_force,
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

//Set PD gains for the haptice device position controller
void PositionControl::setPosCtrlGains(const double kp_position_ctrl_device, const double kv_position_ctrl_device,
                                        const double kp_orientation_ctrl_device, const double kv_orientation_ctrl_device,
                                        const double kp_position_ctrl_gripper, const double kv_position_ctrl_gripper)
{
  _kp_position_ctrl_device = kp_position_ctrl_device;
  _kv_position_ctrl_device = kv_position_ctrl_device;
  _kp_orientation_ctrl_device = kp_orientation_ctrl_device;
  _kv_orientation_ctrl_device = kv_orientation_ctrl_device;
  _kp_position_ctrl_gripper = kp_position_ctrl_gripper;
  _kv_position_ctrl_gripper = kv_position_ctrl_gripper;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Haptic position controller - Additional Methods
////////////////////////////////////////////////////////////////////////////////////////////////////
//Limit desired position to the device workspace boundary
void PositionControl::setWorkspaceLimits(bool virtualWS_OnOff, double device_workspace_radius_limit,
                          double device_workspace_angle_limit)
{
    _add_workspace_virtual_limit = virtualWS_OnOff;
    if (_add_workspace_virtual_limit)
    {
    _device_workspace_radius_limit = device_workspace_radius_limit;
    _device_workspace_angle_limit = device_workspace_angle_limit;
    }
}

// A method to use the gripper like a switch, must be called cyclically
void PositionControl::useGripperAsSwitch(bool& gripper_state, double& commanded_gripper_force_device)
{
    double gripper_start_angle = 10*M_PI/180.0;
    double gripper_switch_angle = 5*M_PI/180.0;
    double gripper_force_click = 3.0;
    double gripper_force_switched = 2.0;

    //Update damping force term from gripper velocity
    double damping_force = -0.05 * _current_gripper_velocity_device;

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


} /* namespace Sai2Primitives */
