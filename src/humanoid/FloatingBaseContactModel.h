/*
 * FloatingBaseContactModel.h
 * 
 * Computes the control oriented grasp matrix Gc, the resultant forces Jacobian Jr 
 * and the controlled forces (contact and internal) jacobian Jf as described in 
 *	Jorda M. Robust Robotic Manipulation for Effective Multi-contact and Safe Physical Interactions (Chapter 4)
 *	USA Stanford 2021
 *	
 *	relies on the environmental contact functionalities of sai2-model
 *	
 *  Created on: March, 2020
 *      Author: Mikael Jorda
 */

#ifndef FLOATINGBASECONTACTMODEL_H_
#define FLOATINGBASECONTACTMODEL_H_

#include "Sai2Model.h"
#include <vector>
// #include <pair>

using namespace std;
using namespace Eigen;

namespace Eigen
{
typedef Matrix<bool, Dynamic, 1> VectorXbool;
typedef Matrix<bool, 3, 1> Vector3bool;
typedef Matrix<bool, 6, 1> Vector6bool;
}

namespace Sai2Primitives
{

class FloatingBaseContactModel
{
public:

	/**
	 * @brief      Constructor
	 *
	 * @param      robot  The robot model
	 */
	FloatingBaseContactModel(Sai2Model::Sai2Model* robot);
	~FloatingBaseContactModel();

	/**
	 * @brief      Computes the size of the matrices. Needs to be called after a
	 *             contact has been added or removed. Reinitializes all the
	 *             contacts to passive and all the internal forces to controlled
	 */
	bool updateStructure();


	/**
	 * @brief      Computes the jacobian matrices
	 */
	bool updateJacobians();

	/**
	 * @brief      Defines which contacts are active, and which internal forces
	 *             are not controlled. By default, all contacts are passive and
	 *             all internal forces are controlled. The inputs are in terms
	 *             of contact link names and vector of booleans to define the
	 *             local different directions in the order x, y, z
	 *
	 * @param[in]  active_contacts_list        Active contacts as a list of
	 *                                         pairs. The first element of the
	 *                                         pair is the contact link name and
	 *                                         the second is a vector of 3
	 *                                         booleans representing the local
	 *                                         x, y and z directions (true for
	 *                                         active, false for passive)
	 * @param[in]  uncontrolled_tensions_list  The uncontrolled tensions as a
	 *                                         list of pairs of contact link
	 *                                         names
	 * @param[in]  active_moments_list         The active moments list defined
	 *                                         as the active contact list
	 * @param[in]  uncontrolled_moments_list   The uncontrolled moments list
	 *                                         defined as the active contact
	 *                                         list
	 *
	 * @return     false if the selection is infeasible or an error occues. true
	 *             otherwise.
	 */
	bool setActiveContacts(const vector<pair<string,Vector3bool>> active_contacts_list,
							const vector<pair<string, string>> uncontrolled_tensions_list,
							const vector<pair<string, Vector3bool>> active_moments_list = vector<pair<string, Vector3bool>>(),
							const vector<pair<string, Vector3bool>> uncontrolled_moments_list = vector<pair<string, Vector3bool>>());

	/**
	 * @brief      Updates the permutation matrices. Uses the order in which the
	 *             contacts were added to the robot model, and the grasp matrix
	 *             definition for ordering. 
	 *             Called automatically at the end of setActiveContacts. 
	 *             The user should not call  this function
	 *
	 * @param[in]  is_active_force               Vector of boolean defining the
	 *                                           active contact forces
	 * @param[in]  is_controlled_internal_force  vector of boolean defining the
	 *                                           controlled internal forces
	 *
	 * @return     success or not
	 */
	bool updatePermutationMatrices(const VectorXbool is_active_force, 
									const VectorXbool is_controlled_internal_force);


	// getter functions
	MatrixXd getGc();
	MatrixXd getJr();
	MatrixXd getJf();


	//////// member variables ////////

	// robot model
	Sai2Model::Sai2Model* _robot;

	// permutation matrices to separate active and passive contacts and controlled and non controlled internal forces
	MatrixXd _P, _Q;
	MatrixXd _G, _Ginv, _G_permuted, _Ginv_permuted, _Gc;
	Vector3d _resolving_point;

	MatrixXd _J_contact;
	MatrixXd _J_ac, _J_pc;

	MatrixXd _J_r, _J_f;

	VectorXbool _is_active_contact;
	VectorXbool _is_controlled_internal_force;

	int _n_contacts;
	int _n_contact_dof;
};

} // namesapce Sai2Primitives

#endif // FLOATINGBASECONTACTMODEL_H_