/*
 * InternalForceSelectionBarrierMethod.h
 *
 *	Class to define contact constraints and compute a suitable 
 *	set of controlled internal forces for robot balance and stability as described in
 *	Jorda M. Robust Robotic Manipulation for Effective Multi-contact and Safe Physical Interactions (Chapter 4)
 *	USA Stanford 2021
 *
 *  Created on: Nov 26, 2019
 *      Author: Mikael Jorda
 */

#ifndef INTERNAL_FORCE_SELECTION_BARRIER_METHOD_H_
#define INTERNAL_FORCE_SELECTION_BARRIER_METHOD_H_

#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

namespace Sai2Primitives
{

class InternalForceSelectionBarrierMethod
{
public:

	/**
	 * @brief      Constructor
	 *
	 * @param[in]  n_contacts          The number of contacts
	 * @param[in]  is_surface_contact  Indicates if each contact is a surface contact or point contact
	 * @param[in]  n_ac_dof            The number of active contacts degree of freedom
	 */
	InternalForceSelectionBarrierMethod(const int n_contacts, const vector<bool> is_surface_contact, const int n_ac_dof = 0);

	/**
	 * @brief      Updates the number of active contacts degree of freedom
	 *
	 * @param[in]  n_ac_dof  The new number of active contacts degree of freedom
	 */
	void updateActiveContacts(const int n_ac_dof);

	/**
	 * @brief      Sets the friction constraint for a given contact
	 *
	 * @param[in]  contact_number  The contact number (from 0 to n_contacts-1)
	 * @param[in]  mu              The friction coefficient (needs to be
	 *                             strictly positive)
	 */
	void setFrictionConstraint(const int contact_number, const double mu);

	/**
	 * @brief      Sets the tilt constraint for a given contact
	 *
	 * @param[in]  contact_number            The contact number (from 0 to
	 *                                       n_contacts-1)
	 * @param[in]  contact_patch_dimmension  The contact patch dimmension in the
	 *                                       form (xmin, xmax, ymin, ymax) in
	 *                                       local contact frame
	 */
	void setTiltConstraint(const int contact_number, const Vector4d contact_patch_dimmension);

	/**
	 * @brief      Sets the rotational friction constraint for a given contact.
	 *             Requires that friction properties are set on that contact
	 *             beforehand
	 *
	 * @param[in]  contact_number                 The contact number
	 * @param[in]  contact_characteristic_length  The contact characteristic length
	 */
	void setRotFricConstraint(const int contact_number, const double contact_characteristic_length);

	/**
	 * @brief      Sets the force minimum (maximum) limit for a contact in a local frame
	 *             direction (0 for x, 1 for y, 2 for z). For example, to set a
	 *             minimum force of 10N in the z direction of the second
	 *             contact, call the function
	 *             setForceMinLimit(1, 2, 10.0)
	 *
	 * @param[in]  contact_number        The contact number
	 * @param[in]  constraint_direction  The constraint direction (0:x, 1:y, 2:z)
	 * @param[in]  limit                 The lower force limit
	 */
	void setForceMinLimit(const int contact_number, const int constraint_direction, const double limit);
	void setForceMaxLimit(const int contact_number, const int constraint_direction, const double limit);
	

	/**
	 * @brief      Sets the joint torque limits.
	 *
	 * @param[in]  constrained_contact_jacobian  The constrained contact jacobian (J_c N_r)
	 * @param[in]  tau_min                       The minimum torques
	 * @param[in]  tau_max                       The maximum torques
	 */
	void setTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max);

	/**
	 * @brief      Prepare everything for the method to run. Needs to be called
	 *             after setting all the constraints and before the control loop
	 *
	 * @param[in]  verbose  
	 */
	void prepareOptimization(const bool verbose = false);

	/**
	 * @brief      Sets an initial guess for the internal forces in order to
	 *             speed up the phase 1 computation. Otherwise, the initial
	 *             guess will be zero
	 *
	 * @param[in]  fi    Internal forces guess
	 */
	void setFiInitialGuess(const VectorXd fi);

	/**
	 * @brief      Updates the torque limits. Call this function in the control
	 *             loop to update the contact Jacobian
	 *
	 * @param[in]  constrained_contact_jacobian  The constrained contact jacobian (J_c N_r)
	 * @param[in]  tau_min                       The minimum torques
	 * @param[in]  tau_max                       The maximum torques
	 */
	void updateTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max);

	/**
	 * @brief      Update the grasp matrix defined by 
	 * 				f_contact = PGc * [fr^T   fac^T   fic^T]^T 
	 * 				Typically, it is defined as P * Gc
	 *
	 * @param[in]  PGc   The grasp matrix as defined above
	 */
	void updateGraspMatrix(const MatrixXd PGc);

	/**
	 * @brief      Updates the resultant force and active contact force as a
	 *             single vector fr_fac^T = [f_r^T     f_ac^T]
	 *
	 * @param[in]  fr_fac  The vector of resultant and active controlled forces
	 */
	void updateFrFac(const VectorXd fr_fac);


	/**
	 * @brief      Compute the total barrier function, its gradient and Hessian
	 *
	 * @param      phi       The total barrier function
	 * @param      grad      The gradient
	 * @param      hess      The hessian
	 * @param[in]  fi_guess  The previous set of controlled internal forces
	 * @param[in]  verbose   
	 *
	 * @return     The phi graduated hess.
	 */
	bool computePhiGradHess(double& phi, VectorXd& grad, MatrixXd& hess, const VectorXd fi_guess, const bool verbose = false);
	/**
	 * @brief      Compute the total barrier function, its gradient and Hessian
	 *             for the phase 1 initialization (minimizes phi + t*s)
	 *
	 * @param      phi       The total barrier function
	 * @param      grad      The gradient
	 * @param      hess      The hessian
	 * @param[in]  fi_guess  The previous set of controlled internal forces
	 * @param[in]  s         
	 * @param[in]  t         
	 * @param[in]  verbose   
	 *
	 * @return     The phi graduated hess phase 1.
	 */
	bool computePhiGradHessPhase1(double& phi, VectorXd& grad, MatrixXd& hess, const VectorXd fi_guess, const double s, const double t, const bool verbose = false);

	double computePhi(const VectorXd fi_guess, const bool verbose = false);
	double computePhiPhase1(const VectorXd fi_guess, const double s, const double t, const bool verbose = false);

	/**
	 * @brief      Sets the meta parameters for phase 1
	 *
	 * @param[in]  t0          Initial t value
	 * @param[in]  mu_phase_1  The increase step for the t value
	 */
	void setMetaParametersPhase1(const double t0, const double mu_phase_1);

	/**
	 * @brief      Sets the backtracking line search parameters for the newton step
	 *
	 * @param[in]  alpha  
	 * @param[in]  beta   
	 */
	void setBacktrackingLineSearchParameters(const double alpha, const double beta);

	bool runPhase1(const double tolerance = 1e-6, const int max_outer_loop_iterations = 100, const int max_inner_loop_iterations = 100);
	
	/**
	 * @brief      Computed the next set of controlled internal forces specification
	 *
	 * @param[in]  tolerance       The tolerance for stopping the newton method
	 * @param[in]  max_iterations  The maximum iterations for the newton method
	 *
	 * @return     success or not
	 */
	bool computeFi(const double tolerance = 1e-6, const int max_iterations = 100);

	/**
	 * @brief      Returns the specified internal forces
	 *
	 * @return     Fi
	 */
	VectorXd getFi();

	///////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////          member variables for internal use        ////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////

	int _n_contacts;                      // number of contacts between robot and environment to consider
	int _n_surface_contacts;
	int _n_ac_dof;
	vector<bool> _is_surface_contact;       // for each one of the contacts, is it a surface contact or point contact
	vector<int> _corresponding_moments;
	int _contact_dof;
	int _fi_dof;

	bool _optimization_ready;

	// MatrixXd _G;                          // grasp matrix at the current configuration, to be set by the user
	MatrixXd _W_bar_extended;                  
	MatrixXd _N;                          
	VectorXd _fr_fac;                         // resultant force to the environment, to be set by the user
	VectorXd _fi_current;                 // the value of the internal forces to be optimized. Initial guess to be set by user
	VectorXd _fc_expected;

	MatrixXd _A_tilt, _A_rfrc, _A_flim, _A_tlim;      // Matrices for the linear constraints
	VectorXd _b_tilt, _b_rfrc, _b_flim, _b_tlim;      // Vectors for the linear constraints

	MatrixXd _S_fric, _S_tilt, _S_rfrc, _S_flim;         // selection matrices for the different constraints
	double _w_fric, _w_tilt, _w_rfrc, _w_flim, _w_tlim;  // weights of the constraints in the optimization

	double _slac_phase_1;

	vector<bool> _has_friction_constraints;
	vector<bool> _has_tilt_constraints;
	vector<bool> _has_rotational_friction_constraints;
	vector<bool> _has_force_limits_constraints;
	bool _has_torque_constraints;

	vector<double> _mu_friction;
	vector<Matrix3d> _R_mu_friction;

	vector<Vector4d> _contact_patch_dimmensions;
	vector<double> _contact_characteristic_length;

	vector<pair<int,pair<int,double>>> _min_force_limits; 
	vector<pair<int,pair<int,double>>> _max_force_limits; 

	MatrixXd _constrained_contact_jacobian;
	VectorXd _tau_min, _tau_max;

	double _t_phase1, _mu_phase1;
	double _alpha_bt_line_search, _beta_bt_line_search;
	int _number_of_inequality_constraint_functions;

};

} // namesapce Sai2Primitives

#endif /* INTERNAL_FORCE_SELECTION_BARRIER_METHOD_H_ */
