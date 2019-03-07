/*
 * OpenLoopTeleop.h
 *
 *      This controller implements the bilateral teleoperation scheme in open loop.
 *	It computes the haptic device force feedback and the controlled robot set position
 *	with respect to the input command of the device. 
 *
 *      Author: Margot
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
	// Constructors
	//------------------------------------------------


	/**
	 * @brief Constructor  This constructor creates the workspace extension controller for a haptic application.
	 *
	 * @param Rmax_dev        BLA
	 *
	 */
	OpenLoopTeleop(cHapticDeviceHandler* handler,
					const int device_index,
					const Eigen::Vector3d centerPos_rob, 
		            const Eigen::Matrix3d centerRot_rob,
		            const Eigen::Matrix3d transformDev_Rob = Eigen::Matrix3d::Identity());

	~OpenLoopTeleop();


	//------------------------------------------------
	// Methods
	//------------------------------------------------

	// -------- core methods --------

	/**
	 * @brief update the haptic device model (mass matrix) (if model defined)
	 * @details This function updates the inertia matrix (Lambda) of the human user with the haptic device. This function uses the robot model and assumes it has been updated.
	 */
	//virtual void updateDeviceModel();


	/**
	 * @brief Computes the haptic device set force and the controlled robot set position
	 * @details Computes the haptic device force and the controlled robot set position with respect to the haptic device positions/velocities, the drift of the workspace and the task force feedback. The controller takes a Vector3d for definition of the device cartesian position and a Matrix3d for the orientation. A VectorXd describes the device velocity and another the desired cartesian force feedback from the task. The haptic device inertia matrix is given in the cartesian space by a Matrix3d.
	 * 
	 * @param Fop_des    The desired cartesian force to apply to the haptic device (F_drift + Fop_task)
	 * @param pos_rob    The desired position of the controlled robot after extension of the workspace
	 * @param rot_rob    The desired orientation of the controlled robot after extension of the workspace
	 * @param pos_op     The position of the haptic device in its operational space
	 * @param rot_op     The orientation of the haptic device in its operational space
	 * @param vel_op     The velocity of the haptic device in its operational space
	 * @param Fop_task   The desired force feedback from the task interaction
	 * @param Lambda     The haptic device and human user mass matrix in the cartesian space
	 */
	void computeHapticCommands_Impedance(
				Eigen::Vector3d& pos_rob,
				Eigen::Matrix3d& rot_rob,
				const Eigen::Vector3d pos_rob_sensed, 
		        const Eigen::Matrix3d rot_rob_sensed);

	void computeHapticCommands_Impedance_PositionOnly(
				Eigen::Vector3d& pos_rob,
				const Eigen::Vector3d pos_rob_sensed);

	void computeHapticCommands_ForceSensor(
				Eigen::Vector3d& pos_rob,
				Eigen::Matrix3d& rot_rob,
				const Eigen::VectorXd f_task_sensed = Eigen::VectorXd::Zero(6),
				const bool filter_on = true);


	/**
	 * @brief      reinitializes the workspace drift to the workspace origin of the controlled robot, the controller parameters are kept as updated, max velocities and drift force/velocity are set back to zero.
	 */
	void reInitializeTask();


	void HomingTask();

	void GravityCompTask();


	// -------- Teleoperation task related methods --------

	/**
	 * @brief Sets the scaling factors between the device workspace and the task environment
	 * @details ...
	 * 
	 * @param Ks        Translational scaling factor
	 * @param KsR       Rotational scaling factor
	 */
	void setScalingFactors(const double Ks, const double KsR);


	void setPosCtrlGains (const double kp_pos, const double kv_pos, const double kp_ori, const double kv_ori);

	void setForceFeedbackCtrlGains (const double k_pos, const double k_ori,
		const Matrix3d Red_factor_rot = Matrix3d::Identity(),
		const Matrix3d Red_factor_trans = Matrix3d::Identity());

	void setFilterCutOffFreq(const double fc_force, const double fc_moment);


	/**
	 * @brief Sets the center of the device workspace
	 * @details The haptic device home position and orientation are set through a Vector3d and a Matrix3d
	 * 
	 * @param HomePos_op     The home position of the haptic device in its operational space
	 * @param HomeRot_op     The home orientation of the haptic device in its operational space
	 */
	void setDeviceCenter(const Eigen::Vector3d HomePos_op, 
		            const Eigen::Matrix3d HomeRot_op = Eigen::Matrix3d::Identity());

/**
	 * @brief Sets the center of the task workspace
	 * @details The robot home position and orientation, with respect to the task, are set through a Vector3d and a Matrix3d
	 * 
	 * @param centerPos_rob     The task home position of the robot in its operational space
	 * @param centerRot_rob     The task home orientation of the robot in its operational space
	 */
	void setRobotCenter(const Eigen::Vector3d centerPos_rob, 
		            const Eigen::Matrix3d centerRot_rob = Eigen::Matrix3d::Identity());


	void EnableGripperUserSwitch();
	bool ReadGripperUserSwitch();

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
	Matrix3d _Red_factor_trans;
	Matrix3d _Red_factor_rot;

	// Sensed force filters
	ButterworthFilter* _force_filter;
	ButterworthFilter* _moment_filter;
	double _fc_force;
	double _fc_moment;

	//Time parameters 
	std::chrono::high_resolution_clock::time_point _t_prev;
	std::chrono::high_resolution_clock::time_point _t_curr;
	std::chrono::duration<double> _t_diff;
	bool _first_iteration;


};


} /* namespace Sai2Primitives */

/* SAI2_PRIMITIVES_HAPTICWSEXT_TASK_H_ */
#endif
