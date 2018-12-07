/**
 * OTG_ori.h
 *
 *	A wrapper to use the Reflexxes library (type II or IV) with Eigen library
 *	specifically to work for orientation using rotation matrices
 *
 * Author: Mikael Jorda
 * Created: October 2018
 */

#ifndef SAI2_COMMON_OTG_ORI_H
#define SAI2_COMMON_OTG_ORI_H

#include <Eigen/Dense>
#include <ReflexxesAPI.h>

#include <iostream>

namespace Sai2Primitives
{

class OTG_ori {
public:


	/**
	 * @brief      constructor
	 *
	 * @param[in]  initial_orientation  The initial orientation to initialize the trajectory generation
	 * @param[in]  loop_time            The duration of a control loop
	 *                                  (typically, 0.001 if the robot is
	 *                                  controlled at 1 kHz)
	 */
	OTG_ori(const Eigen::Matrix3d& initial_orientation, const double loop_time);

	/**
	 * @brief      destructor
	 */
	~OTG_ori();

	void reInitialize(const Eigen::Matrix3d& initial_orientation);

	/**
	 * @brief      Sets the maximum velocity for the trajectory generator
	 *
	 * @param[in]  max_velocity  Vector of the maximum velocity per direction
	 */
	void setMaxVelocity(const Eigen::Vector3d max_velocity);

	/**
	 * @brief      Sets the maximum velocity.
	 *
	 * @param[in]  max_velocity  Scalar of the maximum velocity in all directions
	 */
	void setMaxVelocity(const double max_velocity);

	/**
	 * @brief      Sets the maximum acceleration.
	 *
	 * @param[in]  max_acceleration  Vector of the maximum acceleration
	 */
	void setMaxAcceleration(const Eigen::Vector3d max_acceleration);

	/**
	 * @brief      Sets the maximum acceleration.
	 *
	 * @param[in]  max_acceleration  Scalar of the maximum acceleration
	 */
	void setMaxAcceleration(const double max_acceleration);

	/**
	 * @brief      Sets the maximum jerk.
	 *
	 * @param[in]  max_jerk  Vector of the maximum jerk
	 */
	void setMaxJerk(const Eigen::Vector3d max_jerk);

	/**
	 * @brief      Sets the maximum jerk.
	 *
	 * @param[in]  max_jerk  Scalar of the maximum jerk
	 */
	void setMaxJerk(const double max_jerk);

	/**
	 * @brief      Sets the goal position. The gol velocity will be zero
	 *
	 * @param[in]  goal_orientation  The goal position
	 */
	void setGoalPosition(const Eigen::Matrix3d goal_orientation, const Eigen::Matrix3d current_orientation);

	/**
	 * @brief      Calculates the next desired position and velocity for the
	 *             next step
	 *
	 * @param      next_orientation       The desired orientation in the next step
	 * @param      next_angular_velocity  The desired velocity in the next step
	 */
	void computeNextState(Eigen::Matrix3d& next_orientation, Eigen::Vector3d& next_angular_velocity);

	/**
	 * @brief      Function to know if the goal position and velocity is reached
	 *
	 * @return     true if the goal state is reached, false otherwise
	 */
	bool goalReached();

	double _loop_time;
	bool _goal_reached = false;

	Eigen::Matrix3d _initial_orientation;
	Eigen::Matrix3d _goal_orientation;

	// Reflexxes variables
	int _ResultValue = 0;
    ReflexxesAPI *_RML = NULL;
    RMLPositionInputParameters *_IP = NULL;
    RMLPositionOutputParameters *_OP = NULL;
    RMLPositionFlags _Flags;

};

} /* namespace Sai2Primitives */

#endif //SAI2_COMMON_OTG_ORI_H