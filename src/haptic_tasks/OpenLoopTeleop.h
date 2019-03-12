/*
 * OpenLoopTeleop.h
 *
 *      This controller implements the bilateral teleoperation scheme in open loop.
 *	It computes the haptic device force feedback and the controlled robot set position
 *	with respect to the input command of the device. 
 *
 *      Author: Margot Vulliez & Mikael Jorda
 */

#ifndef SAI2_HAPTIC_TASKS_OPEN_LOOP_TELEOP_H_
#define SAI2_HAPTIC_TASKS_OPEN_LOOP_TELEOP_H_

#include "Sai2Model.h"
#include "chai3d.h"
#include "filters/ButterworthFilter.h"
#include <Eigen/Dense>
#include <string>
#include <chrono>

using namespace std;
using namespace Eigen;
using namespace chai3d;

namespace Sai2Primitives
{

class OpenLoopTeleop
{
public:

	//------------------------------------------------
	// Constructor / Destructor
	//------------------------------------------------


	/**
	 * @brief Constructor  This constructor creates the haptic controller for a simple bilateral teleoperation scheme.
	 *
	 * @param Rmax_dev        BLA
	 *
	 */
	OpenLoopTeleop(cHapticDeviceHandler* handler,
					const int device_index,
					const Eigen::Vector3d centerPos_rob, 
		            const Eigen::Matrix3d centerRot_rob,
		            const Eigen::Matrix3d transformDev_Rob = Eigen::Matrix3d::Identity());

	
	/**
	 * @brief Detructor  This destructor deletes the pointers, stop the haptic controller, and close the haptic device.
	 *
	 *
	 */
	~OpenLoopTeleop();


	//------------------------------------------------
	// Methods
	//------------------------------------------------

	// -------- core methods --------

	/**
	 * @brief Computes the haptic device set force and the controlled robot set position
	 * @details Computes the haptic commands from the sensed task force (proxy=false) or through a stiffness/damping proxy between the set
	 *          and current robot position (proxy=true). When computing the force feedback from the force sensor data, the task force must 
	 * 			be set thanks to updateSensedForce() before calling this function. If the proxy evaluation is used, the current position,
	 * 			rotation matrix, and velocity of the controlled robot are updated with updateSensedRobotPositionVelocity() for this method.
	 * 			The haptic commands can be evaluated in position only (position_ony=true) if only the 3 translational DOFs are controlled and
	 * 			rendered to the user. 'proxy' and 'position_only' flags are gobal variables, set to false by default. 'proxy' selects the
	 * 			calculation mode of the haptic force: via proxy or sensed force. 'position_only' computes the haptic commands in positon only
	 * 
	 * @param pos_rob    		The desired position of the controlled robot
	 * @param rot_rob    		The desired orientation of the controlled robot
	 */
	void computeHapticCommands(Eigen::Vector3d& pos_rob,
								Eigen::Matrix3d& rot_rob);

	/**
	 * @brief Update the sensed force from the task interaction
	 * @details When rendering the sensed force as haptic feedback, this function updates the force sensor data. The global variable
	 * 			'filter_on' enables the filtering of force sensor data. The filter parameters are set thanks to setFilterCutOffFreq().
	 * 
	 * @param f_task_sensed    	Sensed task force from the controlled robot's sensor
	 */
	void updateSensedForce(const Eigen::VectorXd f_task_sensed = Eigen::VectorXd::Zero(6));

	/**
	 * @brief Update the current position and orientation of the controlled robot
	 * @details When evaluating the haptic feedback via an impedance/damping proxy, this function updates the current robot position/rotation.
	 * 
	 * @param pos_rob_sensed    The current position of the controlled robot
	 * @param rot_rob_sensed    The current orientation of the controlled robot
	 */
	void updateSensedRobotPositionVelocity(const Eigen::Vector3d pos_rob_sensed,
											const Eigen::Vector3d vel_rob_trans,
											const Eigen::Matrix3d rot_rob_sensed = Eigen::Matrix3d::Identity(),
											const Eigen::Vector3d vel_rob_rot = Eigen::Vector3d::Zero());

	/**
	 * @brief Set the haptic device in gravity compensation
	 * @details Send zero force/torque feedback to the haptic device.
	 * 
	 */
	void GravityCompTask();

	/**
	 * @brief Place the haptic device in its home position.
	 * @details This method creates a position controller for the haptic device to send it to its home position.
	 * 
	 */
	void HomingTask();

	/**
	 * @brief Use the gripper of the haptic device as a user switch
	 * 
	 */
	void EnableGripperUserSwitch();

	/**
	 * @brief If the gripper is used as a user switch, this methods read the gripper state
	 * 
	 */
	bool ReadGripperUserSwitch();

	/**
	 * @brief Reinitializes the haptic controller parameters to default values.
	 */
	void reInitializeTask();


	// -------- Parameter setting methods --------

	/**
	 * @brief Set the scaling factors between the device workspace and the task environment
	 * 
	 * @param Ks        Translational scaling factor
	 * @param KsR       Rotational scaling factor
	 */
	void setScalingFactors(const double Ks, const double KsR);

	/**
	 * @brief Set the position controller gains of the haptic device for the homing task.
	 * 			The gains are given as a ratio of the device maximum damping and stiffness (between 0 and 1).
	 * 
	 * @param kp_pos    Proportional gain in position  
	 * @param kv_pos    Derivative term in position
	 * @param kp_ori    Proportional gain in orientation
	 * @param kv_ori    Derivative term in orientation
	 */
	void setPosCtrlGains (const double kp_pos, const double kv_pos, const double kp_ori, const double kv_ori);

	/**
	 * @brief Set the impedance/damping terms for the force feedback evaluation via proxy
	 * 		  Define the reduction factors between the actual task force and the rendered force
	 * 
	 * @param k_pos       		Proxy impedance term in position
	 * @param d_pos 	  		Proxy damping term in position
	 * @param k_ori       		Proxy impedance term in orientation
	 * @param d_ori 	  		Proxy damping term in orientation
	 * @param Red_factor_rot 	Matrix of force reduction factors
	 * @param Red_factor_trans 	Matrix of torque reduction factors
	 */
	void setForceFeedbackCtrlGains (const double k_pos, const double d_pos, const double k_ori, const double d_ori,
		const Matrix3d Red_factor_rot = Matrix3d::Identity(),
		const Matrix3d Red_factor_trans = Matrix3d::Identity());

	/**
	 * @brief Set the normalized cut-off frequencies (between 0 and 0.5) to filter the sensed force
	 * 
	 * @param fc_force        Cut-off frequency of the sensed force
	 * @param fc_moment       Cut-off frequency of the sensed torque
	 */
	void setFilterCutOffFreq(const double fc_force, const double fc_moment);

	/**
	 * @brief Set the center of the device workspace
	 * @details The haptic device home position and orientation are set through a Vector3d and a Matrix3d
	 * 
	 * @param HomePos_op     The home position of the haptic device in its operational space
	 * @param HomeRot_op     The home orientation of the haptic device in its operational space
	 */
	void setDeviceCenter(const Eigen::Vector3d HomePos_op, 
		            const Eigen::Matrix3d HomeRot_op = Eigen::Matrix3d::Identity());

	/**
	 * @brief Set the center of the task workspace
	 * @details The robot home position and orientation, with respect to the task, are set through a Vector3d and a Matrix3d
	 * 
	 * @param centerPos_rob     The task home position of the robot in its operational space
	 * @param centerRot_rob     The task home orientation of the robot in its operational space
	 */
	void setRobotCenter(const Eigen::Vector3d centerPos_rob, 
		            const Eigen::Matrix3d centerRot_rob = Eigen::Matrix3d::Identity());

	/**
	 * @brief Set the transformation matrix between the haptic device global frame to the robot global frame
	 * 
	 * @param transformDev_Rob    Rotation matrix between from the device to robot frame
	 */
	void setDeviceRobotTransform(const Eigen::Matrix3d transformDev_Rob = Eigen::Matrix3d::Identity());


	//------------------------------------------------
	// Attributes
	//------------------------------------------------
	
	//// Haptic device handler, status and parameters ////
	// a haptic device handler
	// cHapticDeviceHandler* _handler;
	// a pointer to the current haptic device
	cGenericHapticDevicePtr hapticDevice;
	// Device status
	bool device_started;
	bool device_homed;
	// Device maximum force, damping and stiffness 
	double maxForce_dev;
	double maxTorque_dev;
	double maxLinDamping_dev;
	double maxAngDamping_dev;
	double maxLinStiffness_dev;
	double maxAngStiffness_dev;
	// If gripper used as a switch
	bool gripper_state;
	// Set force and torque feecdback of the haptic device
	Vector3d _force_dev;
	Vector3d _torque_dev;
	cVector3d _force_chai;
	cVector3d _torque_chai;
	// Haptic device home position and orientation
	Vector3d _HomePos_op;
	Matrix3d _HomeRot_op;
	// Haptic device position and rotation
	Vector3d _pos_dev; 
	Matrix3d _rot_dev;
	cVector3d _pos_dev_chai;
	cMatrix3d _rot_dev_chai;
	// Haptic device velocity
	VectorXd _vel_dev_trans;
	VectorXd _vel_dev_rot;
	cVector3d _vel_dev_trans_chai;
	cVector3d _vel_dev_rot_chai;

	// Workspace scaling factors in translation and rotation
	double _Ks, _KsR;

	// Workspace center of the controlled robot in the robot frame
	Vector3d _centerPos_rob;
	Matrix3d _centerRot_rob;

	//Transformation matrix from the device frame to the robot frame
	Matrix3d _transformDev_Rob;
	//Set position and orientation of the controlled robot (in robot frame)
	Vector3d _pos_rob;
	Matrix3d _rot_rob;

	//Position controller parameters
	double _kp_pos;
	double _kv_pos;
	double _kp_ori;
	double _kv_ori;

	// Force feedback controller parameters
	double _k_pos;
	double _k_ori;
	double _d_ori;
	double _d_pos;
	Matrix3d _Red_factor_trans;
	Matrix3d _Red_factor_rot;

	// Sensed force filters
	ButterworthFilter* _force_filter;
	ButterworthFilter* _moment_filter;
	double _fc_force;
	double _fc_moment;
	bool filter_on;

	bool proxy;
	bool position_only;

	// Sensed task force 
	VectorXd _f_task_sensed;

	// Robot current position, rotation, and velocity
	Vector3d _pos_rob_sensed;
	Matrix3d _rot_rob_sensed;
	Vector3d _vel_rob_trans;
	Vector3d _vel_rob_rot;

	// Time parameters 
	chrono::high_resolution_clock::time_point _t_prev;
	chrono::high_resolution_clock::time_point _t_curr;
	chrono::duration<double> _t_diff;
	bool _first_iteration;


};


} /* namespace Sai2Primitives */

/* SAI2_PRIMITIVES_HAPTICWSEXT_TASK_H_ */
#endif
