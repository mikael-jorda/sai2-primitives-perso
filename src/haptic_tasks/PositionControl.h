/*
 * PositionControl.h
 *
 *    This controller implements a simple operational space position controller for the haptic device.
 * 		The command force is computed for the haptic device with respect to the desired position through a PD gain.
 *
 *     Author: Margot Vulliez
 */

 #ifndef SAI2_POSITION_CONTROL_H_
 #define SAI2_POSITION_CONTROL_H_

 #include "Sai2Model.h"
 #include <Eigen/Dense>

 using namespace std;
 using namespace Eigen;

 namespace Sai2Primitives
 {

 class PositionControl
 {

 public:

   ////////////////////////////////////////////////////////////////////////////////////////////////////
   //// Constructor, Destructor and Initialization of the impedance-type controller
   ////////////////////////////////////////////////////////////////////////////////////////////////////
   	/**
   	 * @brief Constructor  This constructor creates the haptic position controller in operational sapce.
   	 * @details:
     *		 Device specifications are used to adjust controller gains and to saturate commanded force/torque to limit values
     *
     * @param max_device_force      Maximal force, torque, and gripper force the device can render
     * @param max_device_stiffness 	Maximal translational and rotational stiffness the device can render
     * @param max_device_damping 		Maximal translational and rotational damping the device can render
   	 *
   	 */
   	PositionControl(const Vector3d max_device_force,
                    const Vector2d max_device_stiffness,
                    const Vector2d max_device_damping);

   	/**
   	 * @brief Detructor  This destructor deletes the pointers and stop the position controller.
   	 */
   	~PositionControl();

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // Haptic position controller - Core Methods (Data updates and commands' computation)
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Computes the commands of the haptic device to follow the desired position
     * @details:
     *			computeCommands - The haptic commands are computed from the PD operational spaces position controller.
     *      computeGripperCommands - The gripper haptic feedback is computed from the desired gripper position
 	   *
 	   *		 !! Make sure to update the haptic device data (position, velocity) before calling this function !!
     *        The desired position is set thanks to setDesiredPosition().
     *        The boolean isInPos returns true if the desied position is reached.
     *        Workspace limits can be checked with setWorkspaceLimits() to limit the position command
 	   *
     * @param commanded_force_device          The desired force feedback of the haptic device
 	   * @param commanded_torque_device         The desired torque feedback of the haptic device
     * @param commanded_gripper_force_device  The desired force applied to the device gripper
 	   */
    void computeCommands(Vector3d& commanded_force_device,
                           Vector3d& commanded_torque_device);

    void computeGripperCommands(double& commanded_gripper_force_device);

    /**
     * @brief Update Haptic device position data (position, rotation, and gripper position), in haptic frame.
      * @details:
      *		 The haptic device sensed position/Rotation and gripper position are updated to compute controller Proportional term.
      *
      * @param current_position_device     		   Haptic device current operational space position
      * @param current_rotation_device    		   Haptic device current operational space rotation matrix
      * @param current_position_gripper_device   Haptic device current gripper position
      */
     void updateDevicePosition(const Vector3d current_position_device = Vector3d::Zero(), const Matrix3d current_rotation_device = Matrix3d::Identity(),
                               const double current_position_gripper_device = 0.0);

     /**
      * @brief Update Haptic device velocity data (translational and rotational device velocities and gripper velocity), in haptic frame.
       * @details:
       *		 The haptic device sensed end-effector and gripper velocities are updated to compute controller Damping term.
       *
       * @param current_trans_velocity_device     Haptic device current end-effector translational velocity
       * @param current_rot_velocity_device       Haptic device current end-effector rotational velocity
       * @param current_gripper_velocity_device   Haptic device current gripper velocity
       */
      void updateDeviceVelocity(const Vector3d current_trans_velocity_device = Vector3d::Zero(), const Vector3d current_rot_velocity_device = Vector3d::Zero(),
                                const double current_gripper_velocity_device = 0.0);


     ////////////////////////////////////////////////////////////////////////////////////////////////////
     // Haptic position controller - Specification Methods (To change set desired values and controller factors)
     ////////////////////////////////////////////////////////////////////////////////////////////////////
     /**
   	 * @brief Set the desired position for the haptic device in operational space
   	 *
   	 * @param desired_position_device             Desired position of the haptic device in its frame
   	 * @param desired_rotation_device             Desired orientation of the haptic device's end-effector in its frame
     * @param desired_gripper_position_device    Desired gripper position for the haptic device
   	 */
   	void setDesiredPosition(const Vector3d desired_position_device = Vector3d::Zero(),
   		            	    const Matrix3d desired_rotation_device = Matrix3d::Identity(),
                        const double desired_gripper_position_device = 0.0);

     /**
   	 * @brief Set the center of the device workspace in device frame, from which workspace boundaries are set
   	 *
   	 * @param center_position_device     Center position of the haptic device in its operational space
   	 * @param center_rotation_device     Center orientation of the haptic device's end-effector in its operational space
   	 */
   	void setDeviceCenter(const Vector3d center_position_device = Vector3d::Zero(),
   		            	    const Matrix3d center_rotation_device = Matrix3d::Identity());

    /**
     * @brief Set haptic device specifications (max stiffness, max damping, and max force/torque/gripper force)
     * @details:
     *		 Device specifications are used to adjust controller gains and to saturate commanded force/torque to limit values
     *
     * @param max_device_force      Maximal force, torque, and gripper force the device can render
     * @param max_device_stiffness 	Maximal translational and rotational stiffness the device can render
     * @param max_device_damping 		Maximal translational and rotational damping the device can render
     */
     void setDeviceMaxSpecification (const Vector3d max_device_force = Vector3d::Zero(),
                                     const Vector2d max_device_stiffness = Vector2d::Zero(),
                                     const Vector2d max_device_damping = Vector2d::Zero());

   /**
    * @brief Set PD gains for the haptice device position controller
    *
    * @param max_device_force      Maximal force, torque, and gripper force the device can render
    * @param max_device_stiffness 	Maximal translational and rotational stiffness the device can render
    * @param max_device_damping 		Maximal translational and rotational damping the device can render
    */
     void setPosCtrlGains (const double kp_position_ctrl_device, const double kv_position_ctrl_device,
                           const double kp_orientation_ctrl_device, const double kv_orientation_ctrl_device,
                           const double kp_position_ctrl_gripper, const double kv_position_ctrl_gripper);


     ////////////////////////////////////////////////////////////////////////////////////////////////////
     // Haptic position controller - Additional Methods
     ////////////////////////////////////////////////////////////////////////////////////////////////////
      /**
    	 * @brief Limit desired position to the device workspace boundary
    	 * @details:
       *    The size of the device workspace is defined by its equivalent sphere radius in [m] and the maximum tilt angle in [rad].
    	 *
       * @param virtualWS_OnOff                  Add virtual workspace limits : true or false
    	 * @param device_workspace_radius_limit   Radius of the smallest sphere including the haptic device Workspace
    	 * @param device_workspace_angle_limit   	Maximum tilt angle of the haptic device
    	 */
    	void setWorkspaceLimits(bool virtualWS_OnOff = false, double device_workspace_radius_limit = 0.1,
                              double device_workspace_angle_limit = 90*M_PI/180.0);

      /**
    	 * @brief A method to use the gripper like a switch, must be called cyclically
    	 * @details:
       *    Return gripper state (boolean value) with respect to gripper position, and send constant force to push back the gripper
    	 *
       * @param gripper_state         Value of gripper switch : true or false
       * @param commanded_gripper_force_device  The desired force applied to the device gripper
    	 */
      void useGripperAsSwitch(bool& gripper_state, double& commanded_gripper_force_device);



///////////////////////////////////////////////////////////////////////////////////
// Controller parameters an data, specifiec through setting methods
///////////////////////////////////////////////////////////////////////////////////
      bool isInPos; // Desired position is reached

private:
      //// Controller status and operational modes ////
      bool _add_workspace_virtual_limit; // add a virtual sphere delimiting the haptic device workspace

      bool gripper_init; // Gripper initialization status

      //Position controller parameters for homing task
      double _kp_position_ctrl_device;
      double _kv_position_ctrl_device;
      double _kp_orientation_ctrl_device;
      double _kv_orientation_ctrl_device;
      double _kp_position_ctrl_gripper;
      double _kv_position_ctrl_gripper;

      //// Haptic device variables ////
    	// Haptic device specifications
    	double _max_linear_stiffness_device, _max_angular_stiffness_device;
    	double _max_linear_damping_device, _max_angular_damping_device;
    	double _max_force_device, _max_torque_device, _max_gripper_force_device;

    	double _device_workspace_radius_limit;
    	double _device_workspace_angle_limit;

      // Desired haptic force
    	Vector3d _commanded_force_device;
    	Vector3d _commanded_torque_device;
    	double _commanded_gripper_force_device;

      // Haptic device desired position
    	Vector3d _desired_position_device;
    	Matrix3d _desired_rotation_device;
    	double _desired_gripper_position_device;

    	// Haptic device position
    	Vector3d _current_position_device;
    	Matrix3d _current_rotation_device;
    	double _current_position_gripper_device;

      // Haptic device velocity
      Vector3d _current_trans_velocity_device;
      Vector3d _current_rot_velocity_device;
      double _current_gripper_velocity_device;

      // Haptic device center position and orientation
      Vector3d _home_position_device;
      Matrix3d _home_rotation_device;

};
}  /* namespace Sai2Primitives */

/* SAI2_IMPEDANCE_CONTROL_H_ */
#endif
