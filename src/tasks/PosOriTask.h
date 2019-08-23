/*
 * PosOriTask.h
 *
 *      This class creates a 6Dof position + orientation hybrid controller for a robotic manipulator using operational space formulation and an underlying PID compensator.
 *      If used for hybrid position force control, assumes a force sensor is attached to the same link as the control frame and the force sensed values are given in sensor frame.
 *      Besides, the force sensed and moment sensed are assumed to be the force and moment that the robot applies to the environment.
 *      It requires a robot model parsed from a urdf file to a Sai2Model object, as well as the definition of a control frame
 *      as a link at which the frame is attached, and an affine transform that determines the position and orientation of the control frame in this link
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI2_PRIMITIVES_POSORI_TASK_H_
#define SAI2_PRIMITIVES_POSORI_TASK_H_

#include "Sai2Model.h"
#include "TemplateTask.h"
#include <Eigen/Dense>
#include <string>
#include <chrono>

#ifdef USING_OTG
	#include "trajectory_generation/OTG_posori.h"
#endif

namespace Sai2Primitives
{

class PosOriTask : public TemplateTask
{
public:

	//------------------------------------------------
	// Constructors
	//------------------------------------------------

	/**
	 * @brief      Constructor that takes an Affine3d matrix for definition of
	 *             the control frame. Creates a full position controller by
	 *             default.
	 *
	 * @param      robot          A pointer to a Sai2Model object for the robot
	 *                            that is to be controlled
	 * @param      link_name      The name of the link in the robot at which to
	 *                            attach the control frame
	 * @param      control_frame  The position and orientation of the control
	 *                            frame in local link coordinates
	 * @param[in]  loop_time      time taken by a control loop. Used only in
	 *                            trajectory generation
	 */
	PosOriTask(Sai2Model::Sai2Model* robot, 
		            const std::string link_name, 
		            const Eigen::Affine3d control_frame = Eigen::Affine3d::Identity(),
		            const double loop_time = 0.001);

	/**
	 * @brief      Constructor that takes a Vector3d for definition of the
	 *             control frame position and a Matrix3d for the frame
	 *             orientation. Creates a full position controller by default.
	 *
	 * @param      robot        A pointer to a Sai2Model object for the robot
	 *                          that is to be controlled
	 * @param      link_name    The name of the link in the robot at which to
	 *                          attach the control frame
	 * @param      pos_in_link  The position the control frame in local link
	 *                          coordinates
	 * @param      rot_in_link  The orientation of the control frame in local
	 *                          link coordinates
	 * @param[in]  loop_time    time taken by a control loop. Used only in
	 *                          trajectory generation
	 */
	PosOriTask(Sai2Model::Sai2Model* robot, 
		            const std::string link_name, 
		            const Eigen::Vector3d pos_in_link, 
		            const Eigen::Matrix3d rot_in_link = Eigen::Matrix3d::Identity(),
		            const double loop_time = 0.001);


	//------------------------------------------------
	// Methods
	//------------------------------------------------

	// -------- core methods --------

	/**
	 * @brief      update the task model (jacobians, task inertia and nullspace
	 *             matrices)
	 * @details    This function updates the jacobian, projected jacobian, task
	 *             inertia matrix (Lambda), dynamically consistent inverse of
	 *             the Jacobian (Jbar) and nullspace matrix of the task N. This
	 *             function uses the robot model and assumes it has been
	 *             updated. There is no use to calling it if the robot
	 *             kinematics or dynamics have not been updated since the last
	 *             call. This function takes the N_prec matrix as a parameter
	 *             which is the product of the nullspace matrices of the higher
	 *             priority tasks. The N matrix will be the matrix to use as
	 *             N_prec for the subsequent tasks. In order to get the
	 *             nullspace matrix of this task alone, one needs to compute _N *
	 *             _N_prec.inverse().
	 *
	 * @param      N_prec  The nullspace matrix of all the higher priority
	 *                     tasks. If this is the highest priority task, use
	 *                     identity of size n*n where n in the number of DoF of
	 *                     the robot.
	 */
	virtual void updateTaskModel(const Eigen::MatrixXd N_prec);

	/**
	 * @brief      Computes the torques associated with this task.
	 * @details    Computes the torques taking into account the last model
	 *             update and updated values for the robot joint
	 *             positions/velocities assumes the desired orientation and
	 *             angular velocity has been updated
	 *
	 * @param      task_joint_torques  the vector to be filled with the new
	 *                                 joint torques to apply for the task
	 */
	virtual void computeTorques(Eigen::VectorXd& task_joint_torques);

	/**
	 * @brief      reinitializes the desired state to the current robot
	 *             configuration as well as the integrator terms
	 */
	void reInitializeTask();

	// -------- force control related methods --------

	/**
	 * @brief      Sets the force sensor frame.
	 *
	 * @param[in]  link_name               The link name on which the sensor is attached
	 * @param[in]  transformation_in_link  The transformation in link of the sensor
	 */
	void setForceSensorFrame(const std::string link_name, const Eigen::Affine3d transformation_in_link);

	/**
	 * @brief      Updates the velues of the sensed force and sensed moment from
	 *             sensor values
	 * @details    Assumes that the sensor is attached to the same link as the
	 *             control frame and that the setSensorFrame finction has been called. 
	 *             The force and moment values given to
	 *             this function are assumed to be in the force sensor frame
	 *             (values taken directly from the force sensor) These values
	 *             are supposed to be the forces that the sensor applies to the
	 *             environment (so the opposite of what the sensor feels)
	 *
	 * @param      sensed_force_sensor_frame   The sensed force as the force
	 *                                         that the sensor applies to the
	 *                                         environment in sensor frame
	 * @param      sensed_moment_sensor_frame  The sensed moment as the moment
	 *                                         that the sensor applies to the
	 *                                         environment in sensor frame
	 */
	void updateSensedForceAndMoment(const Eigen::Vector3d sensed_force_sensor_frame, 
								    const Eigen::Vector3d sensed_moment_sensor_frame);

	/**
	 * @brief      Sets the force controlled axis for a hybrid position force
	 *             controller with 1 DoF force and 2 DoF motion
	 * @details    This is the function to use in order to get the position part
	 *             of the controller behave as a Hybrid Force/Motion controller
	 *             with 1 Dof force. The motion is controlled orthogonally to
	 *             the force. It can be called anytime to change the behavior of
	 *             the controller and reset the integral terms.
	 *
	 * @param      force_axis  The axis in control frame coordinates along which
	 *                         the controller behaves as a force controller.
	 */
	void setForceAxis(const Eigen::Vector3d force_axis);

	/**
	 * @brief      Updates the force controlled axis for a hybrid position force
	 *             controller with 1 DoF force and 2 DoF motion
	 * @details    Use this function in situations when the force axis needs to
	 *             be updated (as an estimated normal for example) over time.
	 *             This does not reset the integral terms. In setting up the
	 *             controller for the first time, prefer setForceAxis.
	 *
	 * @param      force_axis  The axis in control frame coordinates along which
	 *                         the controller behaves as a force controller.
	 */
	void updateForceAxis(const Eigen::Vector3d force_axis);

	/**
	 * @brief      Sets the motion controlled axis for a hybrid position force
	 *             controller with 2 DoF force and 1 DoF motion
	 * @details    This is the function to use in order to get the position part
	 *             of the controller behave as a Hybrid Force/Motion controller
	 *             with 2 Dof force. The motion is controlled along one axis and
	 *             the force is controlled orthogonally to the motion It can be
	 *             called anytime to change the behavior of the controller and
	 *             reset the integral terms.
	 *
	 * @param[in]  motion_axis  The motion axis
	 * @param      force_axis  The axis in control frame coordinates along which the
	 *                         controller behaves as a motion controller.
	 */
	void setLinearMotionAxis(const Eigen::Vector3d motion_axis);

	/**
	 * @brief      Sets the motion controlled axis for a hybrid position force
	 *             controller with 2 DoF force and 1 DoF motion
	 * @details    Use this function in situations when the motion axis needs to
	 *             be updated over time. This does not reset the integral terms.
	 *             In setting up the controller for the first time, prefer
	 *             setMotionAxis.
	 *
	 * @param[in]  motion_axis  The motion axis
	 * @param      force_axis  The axis in control frame coordinates along which the
	 *                         controller behaves as a motion controller.
	 */
	void updateLinearMotionAxis(const Eigen::Vector3d motion_axis);

	/**
	 * @brief      Sets the linear part of the task as a full 3DoF force
	 *             controller
	 * @details    This is the function to use in order to get the position part
	 *             of the controller behave as pure Force controller with 3 Dof
	 *             force. It can be called anytime to change the behavior of the
	 *             controller and reset the integral terms.
	 */
	void setFullForceControl();

	/**
	 * @brief      Sets the linear part of the task as a full 3DoF motion
	 *             controller
	 * @details    This is the function to use in order to get the position part
	 *             of the controller behave as pure Motion controller with 3 Dof
	 *             linear motion. It is de default behavior of the controller.
	 *             It can be called anytime to change the behavior of the
	 *             controller and reset the integral terms.
	 */
	void setFullLinearMotionControl();

	/**
	 * @brief      Sets the moment controlled axis for a hybrid orientation
	 *             moment controller with 1 DoF moment and 2 DoF orientation
	 * @details    This is the function to use in order to get the orientation
	 *             part of the controller behave as a Hybrid Force/Motion
	 *             controller with 1 Dof moment. The rotational motion is
	 *             controlled orthogonally to the moment. 
	 *             
	 *             It can be called anytime to change the behavior of the controller 
	 *             and reset the integral terms.
	 *
	 * @param      moment_axis  The axis in control frame coordinates along
	 *                          which the controller behaves as a moment
	 *                          controller.
	 */
	void setMomentAxis(const Eigen::Vector3d moment_axis);

	/**
	 * @brief      Sets the moment controlled axis for a hybrid orientation
	 *             moment controller with 1 DoF moment and 2 DoF orientation
	 * @details    Use this function in situations when the moment axis needs to
	 *             be updated over time. This does not reset the integral terms.
	 *             
	 *             In setting up the controller for the first time, prefer
	 *             setMomentAxis.
	 *
	 * @param      moment_axis  The axis in control frame coordinates along
	 *                          which the controller behaves as a moment
	 *                          controller.
	 */
	void updateMomentAxis(const Eigen::Vector3d moment_axis);

	/**
	 * @brief      Sets the angular movement controlled axis for a hybrid
	 *             orientation moment controller with 2 DoF moment and 1 DoF
	 *             motion
	 * @details    This is the function to use in order to get the orientation
	 *             part of the controller behave as a Hybrid Force/Motion
	 *             controller with 2 Dof moment. The rotational motion is
	 *             controlled along one axis and the moment is controlled
	 *             orthogonally to the motion.
	 *             
	 *             It can be called anytime to change
	 *             the behavior of the controller and reset the integral terms.
	 *
	 * @param[in]  motion_axis  The motion axis
	 * @param      force_axis  The axis in control frame coordinates along which the
	 *                         controller behaves as a rotational motion controller.
	 */
	void setAngularMotionAxis(const Eigen::Vector3d motion_axis);

	/**
	 * @brief      Sets the angular movement controlled axis for a hybrid
	 *             orientation moment controller with 2 DoF moment and 1 DoF
	 *             motion
	 * @details    Use this function in situations when the angular motion axis
	 *             needs to be updated over time. This does not reset the
	 *             integral terms. 
	 *             
	 *             In setting up the controller for the first
	 *             time, prefer setAngularMotionAxis.
	 *
	 * @param[in]  motion_axis  The motion axis
	 * @param      force_axis  The axis in control frame coordinates along which the
	 *                         controller behaves as a rotational motion controller.
	 */
	void updateAngularMotionAxis(const Eigen::Vector3d motion_axis);

	/**
	 * @brief      Sets the angular part of the task as a full 3DoF moment
	 *             controller
	 * @details    This is the function to use in order to get the orientation
	 *             part of the controller behave as pure moment controller with
	 *             3 Dof moment. It can be called anytime to change the behavior
	 *             of the controller and reset the integral terms.
	 */
	void setFullMomentControl();

	/**
	 * @brief      Sets the angular part of the task as a full 3DoF motion
	 *             controller
	 * @details    This is the function to use in order to get the orientation
	 *             part of the controller behave as pure Motion controller with
	 *             3 Dof angular motion. It is de default behavior of the
	 *             controller. It can be called anytime to change the behavior
	 *             of the controller and reset the integral terms.
	 */
	void setFullAngularMotionControl();

	/**
	 * @brief      Changes the behavior to closed loop force control for the
	 *             force controlled directions in the linear part of the
	 *             controller
	 */
	void setClosedLoopForceControl();

	/**
	 * @brief      Changes the behavior to open loop force control for the force
	 *             controlled directions in the linear part of the controller
	 *             (default behavior)
	 */
	void setOpenLoopForceControl();

	/**
	 * @brief      Changes the behavior to closed loop moment control for the
	 *             moment controlled directions in the angular part of the
	 *             controller
	 */
	void setClosedLoopMomentControl();

	/**
	 * @brief      Changes the behavior to open loop moment control for the
	 *             moment controlled directions in the angular part of the
	 *             controller
	 */
	void setOpenLoopMomentControl();

	// ------- helper methods -------

	/**
	 * @brief      Resets all the integrated errors used in I terms
	 */
	void resetIntegrators();

	/**
	 * @brief      Resets the integrated errors used in I terms for linear part
	 *             of task (position_integrated_error and
	 *             force_integrated_error)
	 */
	void resetIntegratorsLinear();

	/**
	 * @brief      Resets the integrated errors used in I terms for angular part
	 *             of task (orientation_integrated_error and
	 *             moment_integrated_error)
	 */
	void resetIntegratorsAngular();

	//-----------------------------------------------
	//         Member variables
	//-----------------------------------------------

	// inputs to be defined by the user

	// desired pose defaults to the configuration when the task is created
	Eigen::Vector3d _desired_position;           // in robot frame
	Eigen::Matrix3d _desired_orientation;        // in robot frame
	Eigen::Vector3d _desired_velocity;           // in robot frame
	Eigen::Vector3d _desired_angular_velocity;   // in robot frame

	// gains for motion controller
	// defaults to 50 for p gains, 14 for d gains and 0 fir i gains
	double _kp_pos, _kp_ori;
	double _kv_pos, _kv_ori;
	double _ki_pos, _ki_ori;

	// gains for the closed loop force controller
	// by default, the force controller is open loop
	// to set the behavior to closed loop controller, use the functions setClosedLoopForceControl and setClosedLoopMomentControl.
	// the closed loop force controller is a PI controller with feedforward force and velocity based damping.
	// gains default to 1 for p gains, 0.7 for i gains and 10 for d gains
	double _kp_force, _kp_moment;
	double _kv_force, _kv_moment;
	double _ki_force, _ki_moment;
	
	// desired force and moment for the force part of the controller
	// defaults to Zero
	Eigen::Vector3d _desired_force;   // robot frame
	Eigen::Vector3d _desired_moment;  // robot frame

	// velocity saturation is off by default
	bool _use_velocity_saturation_flag;
	double _linear_saturation_velocity;   // defaults to 0.3 m/s
	double _angular_saturation_velocity;  // defaults to PI/3 Rad/s

	bool _use_isotropic_gains;              // defaults to true
	Eigen::Vector3d _kp_pos_vec, _kp_ori_vec;
	Eigen::Vector3d _kv_pos_vec, _kv_ori_vec;
	Eigen::Vector3d _ki_pos_vec, _ki_ori_vec;

	enum DynamicDecouplingType
	{
		FULL_DYNAMIC_DECOUPLING,
		POSITION_ONLY_DYNAMIC_DECOUPLNG,
		NO_DYNAMIC_DECOUPLING
	};

	DynamicDecouplingType _dynamic_decoupling_type;

// trajectory generation via interpolation using Reflexxes Library
// on by defalut
#ifdef USING_OTG
	bool _use_interpolation_flag; 

	// default limits for trajectory generation (same in all directions) :
	// Linear Velocity       - 0.3   m/s
	// Linear Acceleration   - 1.0   m/s^2
	// Linear Jerk           - 3.0   m/s^3
	// Angular Velocity      - PI/3  Rad/s
	// Angular Acceleration  - PI    Rad/s^2
	// Angular Jerk          - 3PI   Rad/s^3
#endif

	// internal variables, not to be touched by the user
	std::string _link_name;
	Eigen::Affine3d _control_frame;   // in link_frame

	// movement quantities
	Eigen::Vector3d _current_position;      // robot frame
	Eigen::Matrix3d _current_orientation;   // robot frame

	Eigen::Vector3d _current_velocity;           // robot frame
	Eigen::Vector3d _current_angular_velocity;   // robot frame

	Eigen::Vector3d _orientation_error;               // robot frame
	Eigen::Vector3d _integrated_orientation_error;    // robot frame
	Eigen::Vector3d _integrated_position_error;       // robot frame
	
	Eigen::Matrix3d _sigma_position;        // control frame
	Eigen::Matrix3d _sigma_orientation;     // control frame

	// force quantities
	Eigen::Affine3d _T_control_to_sensor;  

	Eigen::Vector3d _sensed_force;    // robot frame
	Eigen::Vector3d _sensed_moment;   // robot frame

	Eigen::Vector3d _integrated_force_error;    // robot frame
	Eigen::Vector3d _integrated_moment_error;   // robot frame

	Eigen::Matrix3d _sigma_force;     // control frame
	Eigen::Matrix3d _sigma_moment;    // control frame

	bool _closed_loop_force_control;
	bool _closed_loop_moment_control;

	Eigen::Matrix3d _kp_pos_mat, _kp_ori_mat;
	Eigen::Matrix3d _kv_pos_mat, _kv_ori_mat;
	Eigen::Matrix3d _ki_pos_mat, _ki_ori_mat;

	// model quantities
	Eigen::MatrixXd _jacobian;
	Eigen::MatrixXd _projected_jacobian;
	Eigen::MatrixXd _Lambda;
	Eigen::MatrixXd _Jbar;
	Eigen::MatrixXd _N;

	bool _first_iteration;

	Eigen::Vector3d _step_desired_position;
	Eigen::Vector3d _step_desired_velocity;
	Eigen::Matrix3d _step_desired_orientation;
	Eigen::Vector3d _step_orientation_error;
	Eigen::Vector3d _step_desired_angular_velocity;

#ifdef USING_OTG
	double _loop_time;
	OTG_posori* _otg;
#endif

};

} /* namespace Sai2Primitives */

/* SAI2_PRIMITIVES_POSORI_TASK_H_ */
#endif