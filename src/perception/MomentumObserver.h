/*
 * MomentumObserver.h
 *
 *      Creates a momentum observer to estimate the disturbances torques 
 *      due to unexpected contact on the robot structure as described in 
 *      Haddadin et al. A survey on detection, isolation, and identification. IEEE Transactions on Robotics 33.6 (2017): 1292-1312.
 *
 *      Author: Mikael Jorda
 */

#ifndef MOMENTUMOBSERVER_H_
#define MOMENTUMOBSERVER_H_

#include "Sai2Model.h"
#include <Eigen/Dense>
#include <string>

using namespace std;
using namespace Eigen;

namespace Sai2Primitives
{

class MomentumObserver {
public:

	/**
	 * @brief      Constructor
	 *
	 * @param      robot        The robot model
	 * @param[in]  update_rate  The update rate of the observer (usually the
	 *                          inverse of the control frequency)
	 */
	MomentumObserver(Sai2Model::Sai2Model* robot, double update_rate);

	/**
	 * @brief      Destructor
	 */
	~MomentumObserver();

	/**
	 * @brief      Reinitializes the estimated torques to zero and all the
	 *             internal variables to zero
	 */
	void reInitialize();

	/**
	 * @brief      Sets the gain matrix for the observer (square diagonal matrix)
	 *
	 * @param[in]  K     The new gain
	 */
	void setGain(MatrixXd K);

	/**
	 * @brief      Updates the observer given the command torques for a free
	 *             space task
	 *
	 * @param      command_torques  The command torques
	 */
	void update(VectorXd& command_torques);
	/**
	 * @brief      Updates the observer given the command torques and the task
	 *             contact torques (given by J_task^T F_contact_task) for a contact task
	 *
	 * @param      command_torques        The command torques
	 * @param      known_contact_torques  The known contact torques
	 */
	void update(VectorXd& command_torques, VectorXd& known_contact_torques);

	/**
	 * @brief      Returns the disturbance torque estimate.
	 *
	 * @return     The disturbance torque estimate.
	 */
	VectorXd getDisturbanceTorqueEstimate();

	// internal member variables
	Sai2Model::Sai2Model* _robot;

	MatrixXd _K0;

	VectorXd _beta;
	VectorXd _rho;
	VectorXd _integrated_rho_hat;
	VectorXd _r;
	VectorXd _rho_0;

	double _dt;
};

} // namesapce Sai2Primitives

#endif /* MOMENTUMOBSERVER_H_ */