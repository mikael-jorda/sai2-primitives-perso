/*
 * FloatingBaseContactModel.h
 *
 *  Created on: March, 2020
 *      Author: Mikael Jorda
 */

#include "FloatingBaseContactModel.h"

using namespace std;
using namespace Eigen;

namespace Sai2Primitives
{

FloatingBaseContactModel::FloatingBaseContactModel(Sai2Model::Sai2Model* robot)
{
	_robot = robot;
	updateStructure();
}

FloatingBaseContactModel::~FloatingBaseContactModel(){}


bool FloatingBaseContactModel::updateStructure()
{
	_n_contacts = _robot->_environmental_contacts.size();
	if(_n_contacts < 2)
	{
		cout << "Robot model has less than 2 contact points in FloatingBaseContactModel::updateStructure()" << endl;
		cout << "skipping the update and doing nothing" << endl;
		return false;
	}

	_n_contact_dof = 3*_n_contacts;

	for(int i=0 ; i<_n_contacts ; i++)
	{
		if(_robot->_environmental_contacts[i]._contact_type == Sai2Model::ContactType::SurfaceContact)
		{
			_n_contact_dof += 3;
		}
	}

	_P.setIdentity(_n_contact_dof, _n_contact_dof);
	_Q.setIdentity(_n_contact_dof, _n_contact_dof);

	_is_active_contact = VectorXbool::Zero(_n_contact_dof);
	_is_controlled_internal_force = VectorXbool::Ones(_n_contact_dof-6);

	return updateJacobians();
}


bool FloatingBaseContactModel::updateJacobians()
{
	int n_ac_dof = _is_active_contact.count();
	int n_pc_dof = _n_contact_dof - n_ac_dof;
	int n_ic_dof = _n_contact_dof - 6 - n_ac_dof;
	int n_iuc_dof = n_ac_dof;

	// compute J_contact, J_ac and J_pc
	int robot_dof = _robot->dof();
	// MatrixXd Jtmp_6d = MatrixXd::Zero(6,robot_dof);
	// MatrixXd Jtmp_3d = MatrixXd::Zero(3,robot_dof);

	_J_contact = MatrixXd::Zero(_n_contact_dof, robot_dof);
	int k = 0;
	for(int i=0 ; i<_n_contacts ; i++)
	{
		Sai2Model::ContactModel current_contact = _robot->_environmental_contacts[i];

		// cout << "current contact : " << current_contact._link_name << endl;
		// cout << "current contact pos in link : " << current_contact._contact_position.transpose() << endl;

		if(current_contact._contact_type == Sai2Model::ContactType::SurfaceContact)
		{
			MatrixXd Jtmp_6d = MatrixXd::Zero(6,robot_dof);
			_robot->J_0LocalFrame(Jtmp_6d, current_contact._link_name, current_contact._contact_position, current_contact._contact_orientation);
			// _robot->J_0(Jtmp_6d, current_contact._link_name, current_contact._contact_position);
			_J_contact.block(3*i, 0, 3, robot_dof) = Jtmp_6d.block(0,0,3,robot_dof);
			_J_contact.block(3*(_n_contacts+k), 0, 3, robot_dof) = Jtmp_6d.block(3,0,3,robot_dof);
			k++;
		}
		else
		{
			MatrixXd Jtmp_3d = MatrixXd::Zero(3,robot_dof);
			_robot->JvLocalFrame(Jtmp_3d, current_contact._link_name, current_contact._contact_position, current_contact._contact_orientation);
			// _robot->Jv(Jtmp_3d, current_contact._link_name, current_contact._contact_position);
			_J_contact.block(3*i, 0, 3, robot_dof) = Jtmp_3d;			
		}
	}

	// cout << endl << endl;

	MatrixXd J_contact_permuted = _P.transpose() * _J_contact;

	// grasp matrices and J_r, J_fe, J_fi
	Matrix3d R;
	Vector3d geom_center;
	_robot->environmentalGraspMatrixAtGeometricCenterLocalContactForces(_G, _Ginv, R, _resolving_point);
	// _robot->environmentalGraspMatrixAtGeometricCenter(_G, _Ginv, R, geom_center);

	_G_permuted = _Q * _G * _P;
	_Ginv_permuted = _P.transpose() * _Ginv * _Q.transpose();
	_Gc.setZero(_n_contact_dof, _n_contact_dof);

	_J_r.setZero(6, robot_dof);
	_J_f.setZero(_n_contact_dof - 6, robot_dof);
	MatrixXd G11, G12, G13, G21, G22, G23;
	if(n_ac_dof > 0)
	{
		G11 = _Ginv_permuted.block(0,0,n_ac_dof,6);
		G12 = _Ginv_permuted.block(0,6,n_ac_dof,n_ic_dof);
		G13 = _Ginv_permuted.block(0,6+n_ic_dof,n_ac_dof,n_iuc_dof);

		G21 = _Ginv_permuted.block(n_ac_dof,0,n_pc_dof,6);
		G22 = _Ginv_permuted.block(n_ac_dof,6,n_pc_dof,n_ic_dof);
		G23 = _Ginv_permuted.block(n_ac_dof,6+n_ic_dof,n_pc_dof,n_iuc_dof);

		MatrixXd G13_inv = G13.inverse();


		// cout << "_Ginv_permuted :\n" << _Ginv_permuted << endl;
		// cout << "G11 :\n" << G11 << endl;
		// cout << "G12 :\n" << G12 << endl;
		// cout << "G13 :\n" << G13 << endl;
		// cout << "G21 :\n" << G21 << endl;
		// cout << "G22 :\n" << G22 << endl;
		// cout << "G23 :\n" << G23 << endl;

		_Gc.block(0,6,n_ac_dof,n_ac_dof) = MatrixXd::Identity(n_ac_dof,n_ac_dof);
		_Gc.block(n_ac_dof,0,n_pc_dof,6) = G21 - G23*G13_inv*G11;
		_Gc.block(n_ac_dof,6,n_pc_dof,n_ac_dof) = G23*G13_inv;
		_Gc.block(n_ac_dof,n_ac_dof+6,n_pc_dof,n_ic_dof) = G22 - G23*G13_inv*G12;

		// cout << "Gc block 11 :\n" << _Gc.block(0,0,n_ac_dof,6);
		// cout << "Gc block 12 :\n" << _Gc.block(0,6,n_ac_dof,n_ac_dof);
		// cout << "Gc block 13 :\n" << _Gc.block(0,6+n_ac_dof,n_ac_dof,n_ic_dof);
		// cout << "Gc block 21 :\n" << _Gc.block(n_ac_dof,0,n_pc_dof,6);
		// cout << "Gc block 22 :\n" << _Gc.block(n_ac_dof,6,n_pc_dof,n_ac_dof);
		// cout << "Gc block 23 :\n" << _Gc.block(n_ac_dof,6+n_ac_dof,n_pc_dof,n_ic_dof);

		_J_ac = J_contact_permuted.block(0,0,n_ac_dof,robot_dof);
		_J_pc = J_contact_permuted.block(n_ac_dof,0,n_pc_dof,robot_dof);

		_J_r = (G21.transpose() - G11.transpose()*G13_inv.transpose()*G23.transpose()) * _J_pc;
		_J_f.topRows(n_ac_dof) = _J_ac + G13_inv.transpose()*G23.transpose() * _J_pc;
		_J_f.bottomRows(n_ic_dof) = (G22.transpose() - G12.transpose()*G13_inv.transpose()*G23.transpose()) * _J_pc;
	}
	else
	{
		G21 = _Ginv_permuted.block(0,0,_n_contact_dof,6);
		G22 = _Ginv_permuted.block(0,6,_n_contact_dof,n_ic_dof);

		_Gc = _Ginv_permuted;

		_J_pc = J_contact_permuted;

		_J_r = G21.transpose() * _J_pc;
		_J_f = G22.transpose() * _J_pc;
	}

	return true;
}

bool FloatingBaseContactModel::setActiveContacts(const vector<pair<string, Vector3bool>> active_forces_list,
											const vector<pair<string, string>> uncontrolled_tensions_list,
											const vector<pair<string, Vector3bool>> active_moments_list,
											const vector<pair<string, Vector3bool>> uncontrolled_moments_list)
{

	VectorXbool is_active_contact_tmp = VectorXbool::Zero(_n_contact_dof);
	VectorXbool is_controlled_internal_force_tmp = VectorXbool::Ones(_n_contact_dof - 6);

	// cout << "elements in active contats vector : " << active_forces_list.size() << endl;
	// cout << "elements in tension vector : " << uncontrolled_tensions_list.size() << endl;
	// cout << "elements in intenal moments vector : " << uncontrolled_moments_list.size() << endl;
	// cout << endl;

	// cout << "starting active force treatment" << endl;
	for (vector<pair<string, Vector3bool>>::const_iterator it = active_forces_list.begin() ; it != active_forces_list.end(); ++it)
	{
		int i_current_contact = -1;
		int current_contact_force_starting_dof = 0;
		// int current_contact_moment_starting_dof = 3*_n_contacts;
		Vector3bool active_dofs_current_contact;

		for(int i=0 ; i<_n_contacts ; i++)
		{
			if(_robot->_environmental_contacts[i]._link_name == it->first)
			{
				// cout << "found desired contact point in environmental contact list : " << it->first << endl;
				// cout << "contact index in environemntal contacts : " << i << endl;
				// cout << "contact dofs to make active : " << it->second.transpose() << endl;;
				i_current_contact = i;
				active_dofs_current_contact = it->second;
				break;
			}
			else
			{
				current_contact_force_starting_dof += 3;
			}

		}
		if(i_current_contact == -1)
		{
			throw invalid_argument("link in active_forces_list " + it->first + " is not considered in contact in FloatingBaseContactModel::setActiveContacts()\n");
		}
		// cout << "inserting force contact dofs to be active starting at : " << current_contact_force_starting_dof << endl;
		is_active_contact_tmp.segment<3>(current_contact_force_starting_dof) = active_dofs_current_contact;
	}

	// cout << endl << "starting active moment treatment" << endl;
	for (vector<pair<string, Vector3bool>>::const_iterator it = active_moments_list.begin() ; it != active_moments_list.end(); ++it)
	{
		int i_current_contact = -1;
		int current_contact_moment_starting_dof = 3*_n_contacts;
		Vector3bool active_dofs_current_contact;

		for(int i=0 ; i<_n_contacts ; i++)
		{
			if(_robot->_environmental_contacts[i]._link_name == it->first)
			{
				if(_robot->_environmental_contacts[i]._contact_type != Sai2Model::ContactType::SurfaceContact)
				{
					throw invalid_argument("link in active_moments_list " + it->first + " needs to be in surface contact in FloatingBaseContactModel::setActiveContacts()\n");
				}
				// cout << "found desired contact point in environmental contact list : " << it->first << endl;
				// cout << "contact index in environemntal contacts : " << i << endl;
				// cout << "contact dofs to make active : " << it->second.transpose() << endl;;
				i_current_contact = i;
				active_dofs_current_contact = it->second;
				break;
			}
			else
			{
				if(_robot->_environmental_contacts[i]._contact_type == Sai2Model::ContactType::SurfaceContact)
				{
					current_contact_moment_starting_dof += 3;
				}
			}

		}
		if(i_current_contact == -1)
		{
			throw invalid_argument("link in active_moments_list " + it->first + " is not considered in contact in FloatingBaseContactModel::setActiveContacts()\n");
		}
		// cout << "inserting moment contact dofs to be active starting at : " << current_contact_moment_starting_dof << endl;
		is_active_contact_tmp.segment<3>(current_contact_moment_starting_dof) = active_dofs_current_contact;
	}

	// cout << endl << "starting tensions treatment" << endl;
	VectorXi tensions_before_starting_index;
	int n_tensions = 1;
	if(_n_contacts > 2)
	{
		n_tensions = 3 * (_n_contacts - 2);
		tensions_before_starting_index = VectorXi::Zero(_n_contacts-1);
		tensions_before_starting_index(1) = _n_contacts-1;
		for(int k=2 ; k<_n_contacts-1 ; k++)
		{
			tensions_before_starting_index(k) = tensions_before_starting_index(k-1) + _n_contacts-k;
		}
		// cout << "vector of previous tensions : " << tensions_before_starting_index.transpose() << endl;
	}	
	for (vector<pair<string, string>>::const_iterator it = uncontrolled_tensions_list.begin() ; it != uncontrolled_tensions_list.end(); ++it)
	{
		int i1_tension = -1;
		int i2_tension = -1;
		for(int i=0 ; i<_n_contacts ; i++)
		{
			if(_robot->_environmental_contacts[i]._link_name == it->first)
			{
				// cout << "first link found : " << it->first << endl;
				// cout << "at index : " << i << endl;
				i1_tension = i;
			}
			if(_robot->_environmental_contacts[i]._link_name == it->second)
			{
				// cout << "second link found : " << it->second << endl;
				// cout << "at index : " << i << endl;
				i2_tension = i;
			}			
		}
		// check that the links exist
		if( i1_tension < 0)
		{
			throw invalid_argument("link in uncontrolled_tensions_list " + it->first + " is not in contact in FloatingBaseContactModel::setActiveContacts\n");
		}
		else if( i2_tension < 0)
		{
			throw invalid_argument("link in uncontrolled_tensions_list " + it->second + " is not in contact in FloatingBaseContactModel::setActiveContacts\n");
		}
		else if(i1_tension == i2_tension)
		{
			throw invalid_argument("the links for each pair (internal tensions) need to be different in FloatingBaseContactModel::setActiveContacts()\n");
		}

		// swap the tension indexes  so i1 < i2
		if(i1_tension > i2_tension)
		{
			// cout << "swapping tension indexes" << endl;
			int i_tmp = i1_tension;
			i1_tension = i2_tension;
			i2_tension = i_tmp;
		}

		// find the index of the tension itself in the virtual linkage representation
		int i_tension = 0;
		if(_n_contacts > 2)
		{
			i_tension = tensions_before_starting_index(i1_tension) + i2_tension - i1_tension - 1;
			// cout << "tension index found : " << i_tension << endl;
		}
		is_controlled_internal_force_tmp(i_tension) = false;
	}

	// cout << endl << "starting internal moments treatment" << endl;
	for (vector<pair<string, Vector3bool>>::const_iterator it = uncontrolled_moments_list.begin() ; it != uncontrolled_moments_list.end(); ++it)
	{
		if(_n_contacts == 2)
		{
			throw invalid_argument("Cannot set an uncontrolled internal moment via the function FloatingBaseContactModel::setActiveContacts() in the 2 contact case. Use FloatingBaseContactModel::updatePermutationMatrices() directly instead");
		}

		int i_intenal_moment = -1;
		int internal_moment_starting_index = n_tensions;
		Vector3bool iuc_moments_current_contact;

		for(int i=0 ; i<_n_contacts ; i++)
		{
			if(_robot->_environmental_contacts[i]._link_name == it->first)
			{
				if(_robot->_environmental_contacts[i]._contact_type != Sai2Model::ContactType::SurfaceContact)
				{
					throw invalid_argument("The links at which to set internal moment control needs to be in surface contact in FloatingBaseContactModel::setActiveContacts()\n");
				}
				// cout << "found desired contact point in environmental contact list : " << it->first << endl;
				// cout << "contact index in environemntal contacts : " << i << endl;
				// cout << "contact dofs to make active : " << it->second.transpose() << endl;;
				i_intenal_moment = i;
				iuc_moments_current_contact = it->second;
				break;
			}
			else
			{
				if(_robot->_environmental_contacts[i]._contact_type == Sai2Model::ContactType::SurfaceContact)
				{
					internal_moment_starting_index += 3;
				}
			}

		}
		if(i_intenal_moment == -1)
		{
			throw invalid_argument("link in uncontrolled_moments_list " + it->first + " is not considered in contact in FloatingBaseContactModel::setActiveContacts()\n");
		}
		// cout << "inserting moment contact dofs to be active starting at : " << internal_moment_starting_index << endl;
		is_controlled_internal_force_tmp.segment<3>(internal_moment_starting_index) = Vector3bool::Ones() - iuc_moments_current_contact;

	}

	// cout << "active contacts : " << is_active_contact_tmp.transpose() << endl;
	// cout << "controlled internal : " << is_controlled_internal_force_tmp.transpose() << endl;

	// _is_active_contact = is_active_contact_tmp;
	// _is_controlled_internal_force = is_controlled_internal_force_tmp;

	return updatePermutationMatrices(is_active_contact_tmp, is_controlled_internal_force_tmp);
}

bool FloatingBaseContactModel::updatePermutationMatrices(const VectorXbool is_active_contact, const VectorXbool is_controlled_internal_force)
{
	// _P.setIdentity();
	// _Q.setIdentity();
	int n_ac_dof = is_active_contact.count();
	int n_ic_dof = is_controlled_internal_force.count();
	int n_pc_dof = _n_contact_dof - n_ac_dof;
	int n_iuc_dof = _n_contact_dof - 6 - n_ic_dof;

	if(n_ac_dof != n_iuc_dof)
	{
		cout << "is active contact : " << is_active_contact.transpose() << endl;
		cout << "is controlled internal : " << is_controlled_internal_force.transpose() << endl;

		cout << "\n--------------------------  WARNING  ---------------------------" << endl;
		cout << "number of active contact dofs need to be the same as the number of uncontrolled internal forces/moments dof in FloatingBaseContactModel::updatePermutationMatrices\n";
		cout << "not updating the permutation matrices" << endl;
		cout << "------------------------- END WARNING  -------------------------\n" << endl;
		return false;
	}

	// cout << "n_ac_dof : " << n_ac_dof << endl;
	// cout << "n_pc_dof : " << n_pc_dof << endl;
	// cout << "n_ic_dof : " << n_ic_dof << endl;
	// cout << "n_iuc_dof : " << n_iuc_dof << endl;

	// compute P
	MatrixXd P1 = MatrixXd::Zero(n_ac_dof, _n_contact_dof);
	int tmp_i = 0;
	for(int j=0; j<_n_contact_dof; j++)
	{
		if(is_active_contact(j))
		{
			// cout << "contact : " << j << " is active" << endl;
			P1(tmp_i, j) = 1;
			tmp_i++;
		}
	}
	MatrixXd P2 = MatrixXd::Zero(n_pc_dof, _n_contact_dof);
	tmp_i = 0;
	for(int j=0; j<_n_contact_dof; j++)
	{
		if(!is_active_contact(j))
		{
			// cout << "contact : " << j << " is passive" << endl;
			P2(tmp_i, j) = 1;
			tmp_i++;
		}
	}
	// cout << "P1 :\n" << P1 << endl; 
	// cout << "P2 :\n" << P2 << endl; 
	// cout << "P :\n" << _P << endl; 
	MatrixXd P_transpose_tmp = MatrixXd::Identity(_n_contact_dof, _n_contact_dof);
	P_transpose_tmp.block(0,0,n_ac_dof, _n_contact_dof) = P1;
	P_transpose_tmp.block(n_ac_dof,0,n_pc_dof, _n_contact_dof) = P2;
	// cout << "P :\n" << P_transpose_tmp << endl;
	// cout << endl; 
	// _P = P_transpose.transpose();
	// cout << "P :\n" << _P << endl;
	// cout << endl; 
	// cout << endl; 
	// cout << endl; 

	// compute Q
	MatrixXd Q1 = MatrixXd::Zero(n_ic_dof, _n_contact_dof-6);
	tmp_i = 0;
	for(int j=0; j<_n_contact_dof-6; j++)
	{
		if(is_controlled_internal_force(j))
		{
			Q1(tmp_i, j) = 1;
			tmp_i++;
		}
	}
	MatrixXd Q2 = MatrixXd::Zero(n_iuc_dof, _n_contact_dof-6);
	tmp_i = 0;
	for(int j=0; j<_n_contact_dof-6; j++)
	{
		if(!is_controlled_internal_force(j))
		{
			Q2(tmp_i, j) = 1;
			tmp_i++;
		}
	}
	MatrixXd Q_tmp = MatrixXd::Identity(_n_contact_dof, _n_contact_dof);
	Q_tmp.block(6,6,n_ic_dof,_n_contact_dof-6) = Q1;
	Q_tmp.block(6+n_ic_dof,6,n_iuc_dof,_n_contact_dof-6) = Q2;

	// check that the result is coherent (G13 is invertible)
	Matrix3d R;
	Vector3d geom_center;
	_robot->environmentalGraspMatrixAtGeometricCenterLocalContactForces(_G, _Ginv, R, geom_center);
	// _robot->environmentalGraspMatrixAtGeometricCenter(_G, _Ginv, R, geom_center);

	MatrixXd G_inv_permuted_tmp = P_transpose_tmp * _Ginv * Q_tmp.transpose();
	// _Ginv_permuted = _P.transpose() * _Ginv * _Q.transpose();

	MatrixXd G13 = G_inv_permuted_tmp.block(0, 6 + n_ic_dof, n_ac_dof, n_ac_dof);
	double determinant = G13.determinant();

	// cout << "is active contact : " << is_active_contact.transpose() << endl;
	// cout << "is controlled internal : " << is_controlled_internal_force.transpose() << endl;
	// cout << "P :\n" << P_transpose_tmp.transpose() << endl;
	// cout << "Q : \n" << Q_tmp << endl;
	// cout << "G :\n" << _G << endl;
	// cout << "G permuted :\n" << Q_tmp * _G * P_transpose_tmp.transpose() << endl;
	// cout << "G inv :\n" << _Ginv << endl;
	// cout << "G inv permuted :\n" << G_inv_permuted_tmp << endl;
	// cout << "G13 :\n" << G13 << endl;
	// cout << "determinant : " << determinant << endl; 
	// cout << endl;

	if(abs(determinant) < 1e-3)
	{
		// cout << "P :\n" << P_transpose_tmp.transpose() << endl;
		// cout << "Q :\n" << Q_tmp << endl;
		cout << "is active contact : " << is_active_contact.transpose() << endl;
		cout << "is controlled internal : " << is_controlled_internal_force.transpose() << endl;
		cout << "G13 :\n" << G13 << endl;
		cout << "determinant : " << determinant << endl; 

		cout << "\n--------------------------  WARNING  ---------------------------" << endl;
		cout << "Incompatible combination of active forces and uncontrolled internal forces in FloatingBaseContactModel::updatePermutationMatrices\n";
		cout << "not updating the permutation matrices" << endl;
		cout << "------------------------- END WARNING  -------------------------\n" << endl;
		
		return false;
	}

	// update everything
	_is_active_contact = is_active_contact;
	_is_controlled_internal_force = is_controlled_internal_force;
	_P = P_transpose_tmp.transpose();
	_Q = Q_tmp;

	return true;
}


MatrixXd FloatingBaseContactModel::getGc()
{
	return _Gc;
}

MatrixXd FloatingBaseContactModel::getJr()
{
	return _J_r;
}

MatrixXd FloatingBaseContactModel::getJf()
{
	return _J_f;
}

} // namesapce Sai2Primitives
