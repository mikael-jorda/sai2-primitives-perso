/*
 * InternalForceSelectionBarrierMethod.cpp
 *
 *  Created on: Nov 26, 2019
 *      Author: Mikael Jorda
 */

#include "InternalForceSelectionBarrierMethod.h"
#include <iostream>
#include <exception>

using namespace std;
using namespace Eigen;

namespace Sai2Primitives
{

InternalForceSelectionBarrierMethod::InternalForceSelectionBarrierMethod(const int n_contacts, const vector<bool> is_surface_contact, const int n_ac_dof)
{
	if(n_contacts < 2)
	{
		throw runtime_error("n_contacts needs to be at least 2 in InternalForceSelectionBarrierMethod()\n");
	}
	if(is_surface_contact.size() != n_contacts)
	{
		throw runtime_error("Size of parameter is_surface_contact inconsistent with n_contacts in InternalForceSelectionBarrierMethod()\n");
	}

	_n_contacts = n_contacts;
	_is_surface_contact = is_surface_contact;
	_contact_dof = 3*_n_contacts;
	_n_surface_contacts = 0;

	for(int i=0 ; i<_n_contacts ; i++)
	{
		if(_is_surface_contact[i])
		{
			_corresponding_moments.push_back(_n_surface_contacts);
			_n_surface_contacts += 1;
			_contact_dof += 3;
		}
		else
		{
			_corresponding_moments.push_back(-1);
		}
	}

	_n_ac_dof = n_ac_dof;
	_fi_dof = _contact_dof - 6 - _n_ac_dof;

	// _G.setZero(_contact_dof, _contact_dof);
	_W_bar_extended.setZero(_contact_dof, 6 + _n_ac_dof);
	_N.setZero(_contact_dof, _fi_dof);
	_fr_fac.setZero(6);
	_fi_current.setZero(_fi_dof);

	bool _optimization_ready = false;

	_w_fric = 1.0;
	_w_tilt = 1.0;
	_w_rfrc = 1.0;
	_w_flim = 1.0;
	_w_tlim = 1.0;

	_slac_phase_1 = -1.0;

	vector<bool> init_constraints(_n_contacts,false);
	_has_friction_constraints = init_constraints;
	_has_tilt_constraints = init_constraints;
	_has_rotational_friction_constraints = init_constraints;
	_has_force_limits_constraints = init_constraints;
	_has_torque_constraints = false;

	for(int i=0 ; i<_n_contacts ; i++)
	{
		_mu_friction.push_back(0);
		_R_mu_friction.push_back(Matrix3d::Identity());

		_contact_patch_dimmensions.push_back(Vector4d::Zero());
		_contact_characteristic_length.push_back(0);
	}

	_t_phase1 = 10.0;
	_mu_phase1 = 15.0;
	_alpha_bt_line_search = 0.01;
	_beta_bt_line_search = 0.5;
	_number_of_inequality_constraint_functions = 0;

}

void InternalForceSelectionBarrierMethod::updateActiveContacts(const int n_ac_dof)
{
	_n_ac_dof = n_ac_dof;
	_fi_dof = _contact_dof - 6 - _n_ac_dof;

	// _G.setZero(_contact_dof, _contact_dof);
	_W_bar_extended.setZero(_contact_dof, 6 + _n_ac_dof);
	_N.setZero(_contact_dof, _fi_dof);
	_fr_fac.setZero(6);
	_fi_current.setZero(_fi_dof);
}


void InternalForceSelectionBarrierMethod::setFrictionConstraint(const int contact_number, const double mu)
{
	if(contact_number < 0 || contact_number >= _n_contacts)
	{
		throw runtime_error("invalid contact number in InternalForceSelectionBarrierMethod::setFrictionConstraint(const int contact_number, const double mu)\n");
	}
	if(mu <= 0)
	{
		throw runtime_error("mu needs to be strictly positive in InternalForceSelectionBarrierMethod::setFrictionConstraint(const int contact_number, const double mu)\n");
	}

	if(_has_friction_constraints[contact_number])
	{
		cout << "\n\nWARNING : friction constraint already set on contact " << contact_number << endl;
		cout << "replacing previous friction value " << _mu_friction[contact_number] << endl;
		cout << "with new friction value " << mu << endl;
	}
	else
	{
		_has_friction_constraints[contact_number] = true;
		_number_of_inequality_constraint_functions += 1;
	}

	_mu_friction[contact_number] = mu;
	_R_mu_friction[contact_number](2,2) = -mu*mu;
}

void InternalForceSelectionBarrierMethod::setTiltConstraint(const int contact_number, const Vector4d contact_patch_dimmension)
{
	if(contact_number < 0 || contact_number >= _n_contacts)
	{
		throw runtime_error("invalid contact number in InternalForceSelectionBarrierMethod::setTiltConstraint(const int contact_number, const Vector4d contact_patch_dimmension)\n");
	}
	if(!_is_surface_contact[contact_number])
	{
		throw runtime_error("Cannot put tilt constraints on a point contact in InternalForceSelectionBarrierMethod::setTiltConstraint(const int contact_number, const Vector4d contact_patch_dimmension)\n");
	}
	if(contact_patch_dimmension(0) >= contact_patch_dimmension(1))
	{
		throw runtime_error("xmin needs to be strictly lower than xmax in InternalForceSelectionBarrierMethod::setTiltConstraint(const int contact_number, const Vector4d contact_patch_dimmension)\n");
	}
	if(contact_patch_dimmension(2) >= contact_patch_dimmension(3))
	{
		throw runtime_error("ymin needs to be strictly lower than ymax in InternalForceSelectionBarrierMethod::setTiltConstraint(const int contact_number, const Vector4d contact_patch_dimmension)\n");
	}

	if(_has_tilt_constraints[contact_number])
	{
		cout << "\n\nWARNING : tilt constraint already set on contact " << contact_number << endl;
		cout << "replacing previous contact patch dimmension " << _contact_patch_dimmensions[contact_number].transpose() << endl;
		cout << "with new contact patch dimmension " << contact_patch_dimmension.transpose() << endl;
	}
	else
	{
		_has_tilt_constraints[contact_number] = true;
		_number_of_inequality_constraint_functions += 4;
	}

	_contact_patch_dimmensions[contact_number] = contact_patch_dimmension;
}

void InternalForceSelectionBarrierMethod::setRotFricConstraint(const int contact_number, const double contact_characteristic_length)
{
	if(contact_number < 0 || contact_number >= _n_contacts)
	{
		throw runtime_error("invalid contact number in InternalForceSelectionBarrierMethod::setRotFricConstraint(const int contact_number, const double contact_characteristic_length)\n");
	}
	if(!_is_surface_contact[contact_number])
	{
		throw runtime_error("Cannot put rotational friction constraints on a point contact in InternalForceSelectionBarrierMethod::setRotFricConstraint(const int contact_number, const double contact_characteristic_length)\n");
	}
	if(!_has_friction_constraints[contact_number])
	{
		throw runtime_error("contact needs to have linear friction constraints beforehand in InternalForceSelectionBarrierMethod::setRotFricConstraint(const int contact_number, const double contact_characteristic_length)\n");
	}
	if(contact_characteristic_length <= 0)
	{
		throw runtime_error("contact charecteristic lengts needs to be strictly positive in InternalForceSelectionBarrierMethod::setRotFricConstraint(const int contact_number, const double contact_characteristic_length)\n");
	}

	if(_has_rotational_friction_constraints[contact_number])
	{
		cout << "\n\nWARNING : rotational friction constraint already set on contact " << contact_number << endl;
		cout << "replacing previous contact patch characteristic length " << _contact_characteristic_length[contact_number] << endl;
		cout << "with new contact patch characteristic length " << contact_characteristic_length << endl;
	}
	else
	{
		_has_rotational_friction_constraints[contact_number] = true;
		_number_of_inequality_constraint_functions += 2;
	}

	_contact_characteristic_length[contact_number] = contact_characteristic_length;
}

void InternalForceSelectionBarrierMethod::setForceMinLimit(const int contact_number, const int constraint_direction, const double limit)
{
	if(contact_number < 0 || contact_number >= _n_contacts)
	{
		throw runtime_error("invalid contact number in InternalForceSelectionBarrierMethod::setForceMinLimit(const int contact_number, const int constraint_direction, const double limit)\n");
	}
	if(constraint_direction < 0 || constraint_direction > 2)
	{
		throw runtime_error("constraint direction needs to be 0(Fx), 1(Fy) or 2(Fz) in InternalForceSelectionBarrierMethod::setForceMinLimit(const int contact_number, const int constraint_direction, const double limit)\n");
	}

	for (vector<pair<int,pair<int,double>>>::iterator it = _max_force_limits.begin() ; it != _max_force_limits.end(); ++it)
	{
		if(it->first == contact_number)
		{
			if(it->second.first == constraint_direction)
			{
				if(it->second.second < limit)
				{
					cout << "trying to add a lower limit " << limit << " higher than a pre existing upper limit on contact " << contact_number;
					cout << "in direction " << constraint_direction << endl;
					throw runtime_error("incompatible limit in InternalForceSelectionBarrierMethod::setForceMinLimit(const int contact_number, const int constraint_direction, const double limit)\n");
				}
			}
		}
	}

	_has_force_limits_constraints[contact_number] = true;

	for (vector<pair<int,pair<int,double>>>::iterator it = _min_force_limits.begin() ; it != _min_force_limits.end(); ++it)
	{
		if(it->first == contact_number)
		{
			if(it->second.first == constraint_direction)
			{
				if(it->second.second < limit)
				{
					cout << "\n\nWARNING : trying to add a lower limit " << limit << " higher than a pre existing lower limit on contact " << contact_number;
					cout << "in direction " << constraint_direction << endl;
					cout << "replacing previous limit in InternalForceSelectionBarrierMethod::setForceMinLimit(const int contact_number, const int constraint_direction, const double limit)" << endl;
					it->second.second = limit;
				}
				else
				{
					cout << "\n\nWARNING : trying to add a lower limit " << limit << " lower than a pre existing lower limit on contact " << contact_number;
					cout << "in direction " << constraint_direction << endl;
					cout << "keeping previous limit in InternalForceSelectionBarrierMethod::setForceMinLimit(const int contact_number, const int constraint_direction, const double limit)" << endl;				
				}
				return;
			}
		}
	}
	_number_of_inequality_constraint_functions += 1;
	_min_force_limits.push_back(make_pair(contact_number, make_pair(constraint_direction, limit)));
}

void InternalForceSelectionBarrierMethod::setForceMaxLimit(const int contact_number, const int constraint_direction, const double limit)
{
	if(contact_number < 0 || contact_number >= _n_contacts)
	{
		throw runtime_error("invalid contact number in InternalForceSelectionBarrierMethod::setForceMaxLimit(const int contact_number, const int constraint_direction, const double limit)\n");
	}
	if(constraint_direction < 0 || constraint_direction > 2)
	{
		throw runtime_error("constraint direction needs to be 0(Fx), 1(Fy) or 2(Fz) in InternalForceSelectionBarrierMethod::setForceMaxLimit(const int contact_number, const int constraint_direction, const double limit)\n");
	}

	for (vector<pair<int,pair<int,double>>>::iterator it = _min_force_limits.begin() ; it != _min_force_limits.end(); ++it)
	{
		if(it->first == contact_number)
		{
			if(it->second.first == constraint_direction)
			{
				if(it->second.second > limit)
				{
					cout << "trying to add an upper limit " << limit << " lower than a pre existing lower limit on contact " << contact_number;
					cout << "in direction " << constraint_direction << endl;
					throw runtime_error("incompatible limit in InternalForceSelectionBarrierMethod::setForceMaxLimit(const int contact_number, const int constraint_direction, const double limit)\n");
				}
			}
		}
	}

	_has_force_limits_constraints[contact_number] = true;

	for (vector<pair<int,pair<int,double>>>::iterator it = _max_force_limits.begin() ; it != _max_force_limits.end(); ++it)
	{
		if(it->first == contact_number)
		{
			if(it->second.first == constraint_direction)
			{
				if(it->second.second > limit)
				{
					cout << "\n\nWARNING : trying to add an upper limit " << limit << " lower than a pre existing upper limit on contact " << contact_number;
					cout << "in direction " << constraint_direction << endl;
					cout << "replacing previous limit in InternalForceSelectionBarrierMethod::setForceMaxLimit(const int contact_number, const int constraint_direction, const double limit)" << endl;
					it->second.second = limit;
				}
				else
				{
					cout << "\n\nWARNING : trying to add an upper limit " << limit << " higher than a pre existing upper limit on contact " << contact_number;
					cout << "in direction " << constraint_direction << endl;
					cout << "keeping previous limit in InternalForceSelectionBarrierMethod::setForceMaxLimit(const int contact_number, const int constraint_direction, const double limit)" << endl;				
				}
				return;
			}
		}
	}
	_number_of_inequality_constraint_functions += 1;
	_max_force_limits.push_back(make_pair(contact_number, make_pair(constraint_direction, limit)));	
}

void InternalForceSelectionBarrierMethod::setTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max)
{
	if(constrained_contact_jacobian.cols() != tau_max.size() || constrained_contact_jacobian.cols() != tau_min.size())
	{
		throw runtime_error("Size of input arguments inconsistent with each other in InternalForceSelectionBarrierMethod::setTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max)\n");
	}
	if(constrained_contact_jacobian.rows() != _contact_dof)
	{
		throw runtime_error("Size of contact jacobian inconsistent with number of contact dof in InternalForceSelectionBarrierMethod::setTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max)\n");
	}
	for(int i=0 ; i<tau_max.size() ; i++)
	{
		if(tau_min(i) >= tau_max(i))
		{
			throw runtime_error("tau_min not strictly lower than tau_max in InternalForceSelectionBarrierMethod::setTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max)");
		}
	}

	if(_has_torque_constraints)
	{
		throw runtime_error("Do not call the function InternalForceSelectionBarrierMethod::setTorqueLimits() a second time.\nUse InternalForceSelectionBarrierMethod::updateTorqueLimits() instead\n");
	}
	else
	{
		_has_torque_constraints = true;
		_number_of_inequality_constraint_functions += 2*tau_min.size();
	}

	_constrained_contact_jacobian = constrained_contact_jacobian;
	_tau_min = tau_min;
	_tau_max = tau_max;
}

void InternalForceSelectionBarrierMethod::prepareOptimization(const bool verbose)
{
	int current_contact = 0;
	// linear friction
	int n_fric_contacts = 0;
	for (int i=0 ; i<_n_contacts ; i++)
	{
		if(_has_friction_constraints[i])
		{
			n_fric_contacts += 1;
		}
	}
	_S_fric.setZero(3*n_fric_contacts, _contact_dof);

	current_contact = 0;
	for(int i=0 ; i<_n_contacts ; i++)
	{
		if(_has_friction_constraints[i])
		{
			_S_fric.block<3,3>(3*current_contact, 3*i) = Matrix3d::Identity();
			current_contact++;
		}
	}

	if(verbose)
	{
		cout << "\n---------------\nPREPARING FORCE OPTIMIZATION\n--------------\n" << endl;
		cout << "number of contact points that have friction : " << n_fric_contacts << endl;
		if(n_fric_contacts > 0)
		{
			cout << "S fric : " << _S_fric.rows() << " x " << _S_fric.cols() << "\n" << _S_fric << endl;
		}
		cout << endl;
	}

	// tilt limits
	int n_tilt_contacts = 0;
	for (int i=0 ; i<_n_contacts ; i++)
	{
		if(_has_tilt_constraints[i])
		{
			n_tilt_contacts += 1;
		}
	}
	_S_tilt.setZero(3*n_tilt_contacts, _contact_dof);
	_b_tilt.setZero(4*n_tilt_contacts);
	_A_tilt.setZero(4*n_tilt_contacts, 3*n_tilt_contacts);

	current_contact = 0;
	for(int i=0 ; i<_n_contacts ; i++)
	{
		if(_has_tilt_constraints[i])
		{
			_A_tilt.block<4,3>(4*current_contact, 3*current_contact) << _contact_patch_dimmensions[i](0),  0,  1,
										  				 			   -_contact_patch_dimmensions[i](1),  0, -1,
										 							    _contact_patch_dimmensions[i](2), -1,  0,
																	   -_contact_patch_dimmensions[i](3),  1,  0;
			_b_tilt.segment<4>(4*current_contact) << 0, 0, 0, 0;

			_S_tilt(3*current_contact, 3*i+2) = 1;
			_S_tilt.block<2,2>(3*current_contact+1, 3*(_n_contacts + _corresponding_moments[i])) = Matrix2d::Identity();
			current_contact++;
		}
	}

	if(verbose)
	{
		cout << "number of contact patches that have tilt constraints : " << n_tilt_contacts << endl;
		if(n_tilt_contacts > 0)
		{
			cout << "A tilt : " << _A_tilt.rows() << " x " << _A_tilt.cols() << "\n" << _A_tilt << endl;
			cout << "b tilt : " << _b_tilt.size() << "\n" << _b_tilt.transpose() << endl;
			cout << "S tilt : " << _S_tilt.rows() << " x " << _S_tilt.cols() << "\n" << _S_tilt << endl;
		}
		cout << endl;
	}

	// rot fric limits
	int n_rfrc_contacts = 0;
	for (int i=0 ; i<_n_contacts ; i++)
	{
		if(_has_rotational_friction_constraints[i])
		{
			n_rfrc_contacts += 1;
		}
	}
	_S_rfrc.setZero(2*n_rfrc_contacts, _contact_dof);
	_b_rfrc.setZero(2*n_rfrc_contacts);
	_A_rfrc.setZero(2*n_rfrc_contacts, 2*n_rfrc_contacts);

	current_contact = 0;
	for(int i=0 ; i<_n_contacts ; i++)
	{
		if(_has_rotational_friction_constraints[i])
		{
			_A_rfrc.block<2,2>(2*current_contact, 2*current_contact) <<  -_contact_characteristic_length[i]*_mu_friction[i],  1,
																		-_contact_characteristic_length[i]*_mu_friction[i], -1;
			_b_rfrc.segment<2>(2*current_contact) << 0, 0;
			_S_rfrc(2*current_contact, 3*i+2) = 1;
			_S_rfrc(2*current_contact+1, 3*(_n_contacts + _corresponding_moments[i])+2) = 1;
			current_contact++;
		}
	}	

	if(verbose)
	{
		cout << "number of contact patches that have rotational constraints : " << n_rfrc_contacts << endl;
		if(n_rfrc_contacts > 0)
		{
			cout << "A rfrc : " << _A_rfrc.rows() << " x " << _A_rfrc.cols() << "\n" << _A_rfrc << endl;
			cout << "b rfrc : " << _b_rfrc.size() << "\n" << _b_rfrc.transpose() << endl;
			cout << "S rfrc : " << _S_rfrc.rows() << " x " << _S_rfrc.cols() << "\n" << _S_rfrc << endl;
		}
		cout << endl;
	}

	// force limits
	int n_flim_dof = _min_force_limits.size() + _max_force_limits.size();

	VectorXd contact_dof_has_force_limit = VectorXd::Zero(_contact_dof);
	VectorXd position_of_contact_dof_in_reduced_representation = -1 * VectorXd::Ones(_contact_dof);
	int contact_dof_counter = 0;
	for (vector<pair<int,pair<int,double>>>::iterator it = _min_force_limits.begin() ; it != _min_force_limits.end(); ++it)
	{
		int current_contact_dof = 3*it->first + it->second.first;
		contact_dof_has_force_limit(current_contact_dof) = 1;

		if(position_of_contact_dof_in_reduced_representation(current_contact_dof) == -1)
		{
			position_of_contact_dof_in_reduced_representation(current_contact_dof) = contact_dof_counter;
			contact_dof_counter++;
		}
	}
	for (vector<pair<int,pair<int,double>>>::iterator it = _max_force_limits.begin() ; it != _max_force_limits.end(); ++it)
	{
		int current_contact_dof = 3*it->first + it->second.first;
		contact_dof_has_force_limit(current_contact_dof) = 1;

		if(position_of_contact_dof_in_reduced_representation(current_contact_dof) == -1)
		{
			position_of_contact_dof_in_reduced_representation(current_contact_dof) = contact_dof_counter;
			contact_dof_counter++;
		}
	}
	int n_contact_dof_that_have_limits = contact_dof_has_force_limit.sum();

	_b_flim.setZero(n_flim_dof);
	_A_flim.setZero(n_flim_dof, n_contact_dof_that_have_limits);
	_S_flim.setZero(n_contact_dof_that_have_limits, _contact_dof);

	for(int i=0 ; i<_contact_dof ; i++)
	{
		if(contact_dof_has_force_limit(i) == 1)
		{
			_S_flim(position_of_contact_dof_in_reduced_representation(i) , i) = 1;
		}
	}

	int current_flim_constraint = 0;
	for (vector<pair<int,pair<int,double>>>::iterator it = _min_force_limits.begin() ; it != _min_force_limits.end(); ++it)
	{
		int current_contact_dof = 3*it->first + it->second.first;
		_b_flim(current_flim_constraint) = -it->second.second;
		_A_flim(current_flim_constraint, position_of_contact_dof_in_reduced_representation(current_contact_dof)) = -1;
		current_flim_constraint++;
	}
	for (vector<pair<int,pair<int,double>>>::iterator it = _max_force_limits.begin() ; it != _max_force_limits.end(); ++it)
	{
		int current_contact_dof = 3*it->first + it->second.first;
		_b_flim(current_flim_constraint) = it->second.second;
		_A_flim(current_flim_constraint, position_of_contact_dof_in_reduced_representation(current_contact_dof)) = 1;
		current_flim_constraint++;
	}

	if(verbose)
	{
		cout << "number of force limit constraints : " << n_flim_dof << endl;
		if(n_flim_dof > 0)
		{
			cout << "number of dofs that have a constraint : " << n_contact_dof_that_have_limits << endl;
			cout << "contact dof that have force limits : " << contact_dof_has_force_limit.transpose() << endl;
			cout << "contact dof position in reduced representation : " << position_of_contact_dof_in_reduced_representation.transpose() << endl;
			cout << "A flim : " << _A_flim.rows() << " x " << _A_flim.cols() << "\n" << _A_flim << endl;
			cout << "b flim : " << _b_flim.size() << "\n" << _b_flim.transpose() << endl;
			cout << "S flim : " << _S_flim.rows() << " x " << _S_flim.cols() << "\n" << _S_flim << endl;
		}
		cout << endl;
	}

	// torque limits
	if(_has_torque_constraints)
	{
		int actuated_dof = _tau_max.size();
		_b_tlim.setZero(2 * actuated_dof);
		_A_tlim.setZero(2 * actuated_dof, _contact_dof);

		_b_tlim.head(actuated_dof) = - _tau_min;
		_b_tlim.tail(actuated_dof) = _tau_max;
		_A_tlim.block(0,0,actuated_dof, _contact_dof) = -_constrained_contact_jacobian.transpose();
		_A_tlim.block(actuated_dof,0,actuated_dof, _contact_dof) = _constrained_contact_jacobian.transpose();

		if(verbose)
		{
			cout << "torque constraints present" << endl;
			cout << "actuated dofs : " << actuated_dof << endl;
			cout << "A tlim : " << _A_tlim.rows() << " x " << _A_tlim.cols() << "\n" << _A_tlim << endl;
			cout << "b tlim : " << _b_tlim.size() << "\n" << _b_tlim.transpose() << endl;	
		}	
	}

	if(verbose)
	{
		if(!_has_torque_constraints)
		{
			cout << "no torque constraints" << endl;
		}
		cout << "\nnumber of inequality constraints : " << _number_of_inequality_constraint_functions << endl;
		cout << "\n---------------\nEND OF FORCE OPTIMIZATION PREPARATION\n--------------\n" << endl;
	}

	_optimization_ready = true;

}

void InternalForceSelectionBarrierMethod::setFiInitialGuess(const VectorXd fi)
{
	if(fi.size() != _fi_dof)
	{
		throw runtime_error("size of input parameter inconsistent with fi dof in InternalForceSelectionBarrierMethod::setFiInitialGuess(const VectorXd fi)\n");
	}

	_fi_current = fi;
}

void InternalForceSelectionBarrierMethod::updateTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max)
{
	if(constrained_contact_jacobian.cols() != tau_max.size() || constrained_contact_jacobian.cols() != tau_min.size())
	{
		throw runtime_error("Size of input arguments inconsistent with each other in InternalForceSelectionBarrierMethod::setTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max)\n");
	}
	if(constrained_contact_jacobian.rows() != _contact_dof)
	{
		throw runtime_error("Size of contact jacobian inconsistent with number of contact dof in InternalForceSelectionBarrierMethod::setTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max)\n");
	}
	int actuated_dof = tau_max.size();

	if(actuated_dof != _tau_max.size())
	{
		throw runtime_error("Size of tau_max not consistent with pre existing value in InternalForceSelectionBarrierMethod::updateTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max)\n");
	}

	for(int i=0 ; i<tau_max.size() ; i++)
	{
		if(tau_min(i) >= tau_max(i))
		{
			throw runtime_error("tau_min not strictly lower than tau_max in InternalForceSelectionBarrierMethod::setTorqueLimits(const MatrixXd constrained_contact_jacobian, const VectorXd tau_min, const VectorXd tau_max)");
		}
	}	

	_constrained_contact_jacobian = constrained_contact_jacobian;
	_tau_min = tau_min;
	_tau_max = tau_max;

	_b_tlim.head(actuated_dof) = - _tau_min;
	_b_tlim.tail(actuated_dof) = _tau_max;
	_A_tlim.block(0,0,actuated_dof, _contact_dof) = -_constrained_contact_jacobian.transpose();
	_A_tlim.block(actuated_dof,0,actuated_dof, _contact_dof) = _constrained_contact_jacobian.transpose();
}

void InternalForceSelectionBarrierMethod::updateGraspMatrix(const MatrixXd PGc)
{
	int G_size = PGc.rows();
	if(PGc.cols() != G_size)
	{
		throw runtime_error("PGc must be a square matrix in InternalForceSelectionBarrierMethod::updateGraspMatrix()\n");
	}
	if(G_size != _contact_dof)
	{
		throw runtime_error("Size of PGc incompatible with number of contact dof in InternalForceSelectionBarrierMethod::updateGraspMatrix()\n");
	}

	// _G = PGc;
	// MatrixXd G_inv = _G.inverse();

	_W_bar_extended = PGc.block(0, 0, _contact_dof, 6 + _n_ac_dof);
	_N = PGc.block(0, 6 + _n_ac_dof, _contact_dof, _contact_dof - 6 -_n_ac_dof);
}

void InternalForceSelectionBarrierMethod::updateFrFac(const VectorXd fr_fac)
{
	if(fr_fac.size() != 6 + _n_ac_dof)
	{
		throw runtime_error("size of fr_fac must be 6 + active contact DoF in InternalForceSelectionBarrierMethod::updateFrFac()\n");
	}

	_fr_fac = fr_fac;
}

bool InternalForceSelectionBarrierMethod::computePhiGradHess(double& phi, VectorXd& grad, MatrixXd& hess, const VectorXd fi_guess, const bool verbose)
{
	if(verbose)
	{
		cout << "start InternalForceSelectionBarrierMethod::computePhiGradHess()" << endl;
	}
	phi = 0;
	grad = VectorXd::Zero(fi_guess.size());
	hess = MatrixXd::Zero(fi_guess.size(), fi_guess.size());
	VectorXd grad_tmp = VectorXd::Zero(fi_guess.size()+1);
	MatrixXd hess_tmp = MatrixXd::Zero(fi_guess.size()+1, fi_guess.size()+1);

	if(computePhiGradHessPhase1(phi, grad_tmp, hess_tmp, fi_guess, 0, 0, verbose))
	{
		grad = grad_tmp.head(fi_guess.size());
		hess = hess_tmp.block(0,0,fi_guess.size(),fi_guess.size());
		if(verbose)
		{
			cout << "grad\n" << grad.transpose() << endl;
			cout << "hess\n" << hess << endl;
			cout << "finished InternalForceSelectionBarrierMethod::computePhiGradHess()" << endl;
			cout << endl;
		}
		return true;
	}
	else
	{
		if(verbose)
		{
			cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiGradHess()" << endl;
			cout << endl;
		}
		return false;
	}
}


bool InternalForceSelectionBarrierMethod::computePhiGradHessPhase1(double& phi, VectorXd& grad, MatrixXd& hess, const VectorXd fi_guess, const double s, const double t, const bool verbose)
{
	if(verbose)
	{
		cout << "start InternalForceSelectionBarrierMethod::computePhiGradHessPhase1()" << endl;
	}
	VectorXd fc = _W_bar_extended*_fr_fac + _N*fi_guess;
	phi = 0;
	VectorXd grad_fc = VectorXd::Zero(fc.size()+1);
	MatrixXd hess_fc = MatrixXd::Zero(fc.size()+1, fc.size()+1);
	grad = VectorXd::Zero(fi_guess.size()+1);
	hess = MatrixXd::Zero(fi_guess.size()+1, fi_guess.size()+1);
	
	// cout << "\n\nCompute phi grad hess phase 1 :\n\n" << endl;
	// cout << "Wbare :\n" << _W_bar_extended << endl;
	// cout << "N :\n" << _N << endl;
	// cout << "fr_fac : " << _fr_fac.transpose() << endl;
	// cout << "fi_guess : " << fi_guess.transpose() << endl;
	// cout << "fc : " << fc.transpose() << endl;

	// augment the selection matrices and the contact force
	MatrixXd S_fric_aug = MatrixXd::Zero(_S_fric.rows()+1, _S_fric.cols()+1);
	S_fric_aug.block(0,0,_S_fric.rows(),_S_fric.cols()) = _S_fric;
	S_fric_aug(_S_fric.rows(),_S_fric.cols()) = 1;

	MatrixXd S_tilt_aug = MatrixXd::Zero(_S_tilt.rows()+1, _S_tilt.cols()+1);
	S_tilt_aug.block(0,0,_S_tilt.rows(),_S_tilt.cols()) = _S_tilt;
	S_tilt_aug(_S_tilt.rows(),_S_tilt.cols()) = 1;

	MatrixXd S_rfrc_aug = MatrixXd::Zero(_S_rfrc.rows()+1, _S_rfrc.cols()+1);
	S_rfrc_aug.block(0,0,_S_rfrc.rows(),_S_rfrc.cols()) = _S_rfrc;
	S_rfrc_aug(_S_rfrc.rows(),_S_rfrc.cols()) = 1;

	MatrixXd S_flim_aug = MatrixXd::Zero(_S_flim.rows()+1, _S_flim.cols()+1);
	S_flim_aug.block(0,0,_S_flim.rows(),_S_flim.cols()) = _S_flim;
	S_flim_aug(_S_flim.rows(),_S_flim.cols()) = 1;

	VectorXd fc_aug = VectorXd::Zero(fc.size()+1);
	fc_aug.head(fc.size()) = fc;
	fc_aug(fc.size()) = s;

	// linear friction
	VectorXd fc_fric = _S_fric * fc;
	const int i_linear_friction_max = count(_has_friction_constraints.begin(), _has_friction_constraints.end(), true);
	
	if(i_linear_friction_max > 0)
	{
		int i_linear_friction = 0;
		VectorXd grad_fric = VectorXd::Zero(3*i_linear_friction_max+1);
		MatrixXd hess_fric = MatrixXd::Zero(3*i_linear_friction_max+1, 3*i_linear_friction_max+1);

		for(int j=0 ; j<_n_contacts ; j++)
		{
			if(_has_friction_constraints[j])
			{
				Vector3d fc_fric_current = fc_fric.segment<3>(3*i_linear_friction);
				double gi = -fc_fric_current.dot(_R_mu_friction[j]*fc_fric_current);
				if(s + gi <= 0)
				{
					if(verbose)
					{
						cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiGradHessPhase1()" << endl;
						cout << endl;
					}
					return false;
				}
				else
				{
					phi -= _w_fric * log(s + gi);
					Vector3d gradi = 2.0/(s + gi)*_R_mu_friction[j]*fc_fric_current;

					grad_fric.segment<3>(3*i_linear_friction) = gradi;
					grad_fric(3*i_linear_friction_max) -= 1.0 / (s+gi);

					hess_fric.block<3,3>(3*i_linear_friction,3*i_linear_friction) = gradi * gradi.transpose() + 2.0/(s + gi)*_R_mu_friction[j];
					hess_fric.block<3,1>(3*i_linear_friction,3*i_linear_friction_max) = -2.0/((s+gi)*(s+gi))*_R_mu_friction[j]*fc_fric_current;
					hess_fric.block<1,3>(3*i_linear_friction_max,3*i_linear_friction) = -2.0/((s+gi)*(s+gi))*(_R_mu_friction[j]*fc_fric_current).transpose();
					hess_fric(3*i_linear_friction_max, 3*i_linear_friction_max) += 1.0/((s+gi)*(s+gi));
				}
				i_linear_friction++;
			}
		}

		grad_fc += _w_fric * S_fric_aug.transpose() * grad_fric;
		hess_fc += _w_fric * S_fric_aug.transpose() * hess_fric * S_fric_aug;

		if(verbose)
		{
			cout << "grad fric\n" << grad_fric.transpose() << endl;
			cout << "hess fric\n" << hess_fric << endl;
			// cout << endl;
		}
	}

	// tilt
	bool consider_tilt = _contact_patch_dimmensions.size() > 0;

	if(consider_tilt)
	{
		MatrixXd A_tilt_aug = MatrixXd::Zero(_A_tilt.rows(), _A_tilt.cols()+1);
		A_tilt_aug.block(0,0,_A_tilt.rows(),_A_tilt.cols()) = _A_tilt;
		A_tilt_aug.block(0,_A_tilt.cols(),_A_tilt.rows(),1) = -VectorXd::Ones(_A_tilt.rows());
		VectorXd fc_tilt_aug = S_tilt_aug * fc_aug;

		VectorXd r_tilt = _b_tilt - A_tilt_aug*fc_tilt_aug;
		VectorXd d_tilt = VectorXd::Zero(r_tilt.size());
		MatrixXd D_tilt = MatrixXd::Zero(r_tilt.size(), r_tilt.size());

		for(int i=0 ; i<r_tilt.size() ; i++)
		{
			if(r_tilt(i) < 0)
			{
				if(verbose)
				{
					cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiGradHessPhase1()" << endl;
					cout << endl;
				}
				return false;
			}
			else
			{
				phi -= log(r_tilt(i));
				d_tilt(i) = -1.0/r_tilt(i);
				D_tilt(i,i) = 1.0/(r_tilt(i)*r_tilt(i));
			}
		}

		VectorXd grad_tilt = -A_tilt_aug.transpose() * d_tilt;
		MatrixXd hess_tilt = A_tilt_aug.transpose() * D_tilt * A_tilt_aug;

		grad_fc += _w_tilt * S_tilt_aug.transpose() * grad_tilt;
		hess_fc += _w_tilt * S_tilt_aug.transpose() * hess_tilt * S_tilt_aug;

		if(verbose)
		{		
			cout << "grad tilt\n" << grad_tilt.transpose() << endl;
			cout << "hess tilt\n" << hess_tilt << endl;
			// cout << endl;
		}
	}

	// rotational friction
	bool consider_rfrc = _contact_characteristic_length.size() > 0;
	if(consider_rfrc)
	{
		MatrixXd A_rfrc_aug = MatrixXd::Zero(_A_rfrc.rows(), _A_rfrc.cols()+1);
		A_rfrc_aug.block(0,0,_A_rfrc.rows(),_A_rfrc.cols()) = _A_rfrc;
		A_rfrc_aug.block(0,_A_rfrc.cols(),_A_rfrc.rows(),1) = -VectorXd::Ones(_A_rfrc.rows());
		VectorXd fc_rfrc_aug = S_rfrc_aug * fc_aug;

		VectorXd r_rfrc = _b_rfrc - A_rfrc_aug*fc_rfrc_aug;
		VectorXd d_rfrc = VectorXd::Zero(r_rfrc.size());
		MatrixXd D_rfrc = MatrixXd::Zero(r_rfrc.size(), r_rfrc.size());

		for(int i=0 ; i<r_rfrc.size() ; i++)
		{
			if(r_rfrc(i) < 0)
			{
				if(verbose)
				{
					cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiGradHessPhase1()" << endl;
					cout << endl;
				}
				return false;
			}
			else
			{
				phi -= log(r_rfrc(i));
				d_rfrc(i) = -1.0/r_rfrc(i);
				D_rfrc(i,i) = 1.0/(r_rfrc(i)*r_rfrc(i));
			}
		}

		VectorXd grad_rfrc = -A_rfrc_aug.transpose() * d_rfrc;
		MatrixXd hess_rfrc = A_rfrc_aug.transpose() * D_rfrc * A_rfrc_aug;

		grad_fc += _w_rfrc * S_rfrc_aug.transpose() * grad_rfrc;
		hess_fc += _w_rfrc * S_rfrc_aug.transpose() * hess_rfrc * S_rfrc_aug;

		if(verbose)
		{
			cout << "grad rfrc\n" << grad_rfrc.transpose() << endl;
			cout << "hess rfrc\n" << hess_rfrc << endl;
			// cout << endl;
		}
	}

	// force limits
	bool consider_flim = _min_force_limits.size() > 0 || _max_force_limits.size() > 0;
	if(consider_flim)
	{
		MatrixXd A_flim_aug = MatrixXd::Zero(_A_flim.rows(), _A_flim.cols()+1);
		A_flim_aug.block(0,0,_A_flim.rows(),_A_flim.cols()) = _A_flim;
		A_flim_aug.block(0,_A_flim.cols(),_A_flim.rows(),1) = -VectorXd::Ones(_A_flim.rows());
		VectorXd fc_flim_aug = S_flim_aug * fc_aug;

		VectorXd r_flim = _b_flim - A_flim_aug*fc_flim_aug;
		VectorXd d_flim = VectorXd::Zero(r_flim.size());
		MatrixXd D_flim = MatrixXd::Zero(r_flim.size(), r_flim.size());

		for(int i=0 ; i<r_flim.size() ; i++)
		{
			if(r_flim(i) < 0)
			{
				if(verbose)
				{
					cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiGradHessPhase1()" << endl;
					cout << endl;
				}
				return false;
			}
			else
			{
				phi -= log(r_flim(i));
				d_flim(i) = -1.0/r_flim(i);
				D_flim(i,i) = 1.0/(r_flim(i)*r_flim(i));
			}
		}

		VectorXd grad_flim = -A_flim_aug.transpose() * d_flim;
		MatrixXd hess_flim = A_flim_aug.transpose() * D_flim * A_flim_aug;

		grad_fc += _w_flim * S_flim_aug.transpose() * grad_flim;
		hess_fc += _w_flim * S_flim_aug.transpose() * hess_flim * S_flim_aug;

		if(verbose)
		{
			cout << "grad flim\n" << grad_flim.transpose() << endl;
			cout << "hess flim\n" << hess_flim << endl;
			// cout << endl;
		}
	}

	// torque limits
	if(_has_torque_constraints)
	{
		MatrixXd A_tlim_aug = MatrixXd::Zero(_A_tlim.rows(), _A_tlim.cols()+1);
		A_tlim_aug.block(0,0,_A_tlim.rows(),_A_tlim.cols()) = _A_tlim;
		A_tlim_aug.block(0,_A_tlim.cols(),_A_tlim.rows(),1) = -VectorXd::Ones(_A_tlim.rows());
		VectorXd fc_tlim_aug = fc_aug;

		VectorXd r_tlim = _b_tlim - A_tlim_aug*fc_tlim_aug;
		VectorXd d_tlim = VectorXd::Zero(r_tlim.size());
		MatrixXd D_tlim = MatrixXd::Zero(r_tlim.size(), r_tlim.size());

		for(int i=0 ; i<r_tlim.size() ; i++)
		{
			if(r_tlim(i) < 0)
			{
				if(verbose)
				{
					cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiGradHessPhase1()" << endl;
					cout << endl;
				}
				return false;
			}
			else
			{
				phi -= log(r_tlim(i));
				d_tlim(i) = -1.0/r_tlim(i);
				D_tlim(i,i) = 1.0/(r_tlim(i)*r_tlim(i));
			}
		}

		VectorXd grad_tlim = -A_tlim_aug.transpose() * d_tlim;
		MatrixXd hess_tlim = A_tlim_aug.transpose() * D_tlim * A_tlim_aug;

		grad_fc += _w_tlim *grad_tlim;
		hess_fc += _w_tlim * hess_tlim;

		if(verbose)
		{
			cout << "grad tlim\n" << grad_tlim.transpose() << endl;
			cout << "hess tlim\n" << hess_tlim << endl;
			// cout << endl;
		}
	}

	MatrixXd N_aug = MatrixXd::Zero(_N.rows()+1, _N.cols()+1);
	N_aug.block(0,0,_N.rows(),_N.cols()) = _N;
	N_aug(_N.rows(),_N.cols()) = 1;

	grad = N_aug.transpose() * grad_fc;
	hess = N_aug.transpose() * hess_fc * N_aug;

	phi += t*s;
	grad(_fi_dof) += t;

	if(verbose)
	{
		cout << "grad\n" << grad.transpose() << endl;
		cout << "hess\n" << hess << endl;
		cout << "finished InternalForceSelectionBarrierMethod::computePhiGradHessPhase1()" << endl;
		cout << endl;
	}

	return true;
}


double InternalForceSelectionBarrierMethod::computePhi(const VectorXd fi_guess, const bool verbose)
{
	if(verbose)
	{
		cout << "computePhi() starts " << endl;
	}
	return computePhiPhase1(fi_guess, 0, 0, verbose);
}

double InternalForceSelectionBarrierMethod::computePhiPhase1(const VectorXd fi_guess, const double s, const double t, const bool verbose)
{
	VectorXd fc = _W_bar_extended*_fr_fac + _N*fi_guess;
	_fc_expected = fc;
	double phi = 0;

	if(verbose)
	{
		cout << "computePhiPhase1() starts " << endl;
		cout << "fr fac current : " << _fr_fac.transpose() << endl;
		cout << "fi guess : " << fi_guess.transpose() << endl;
		cout << "fc computed : " << fc.transpose() << endl;
		// cout << "_W_bar_extended :\n" << _W_bar_extended << endl;
		// cout << "_N :\n" << _N << endl;
		// cout << endl;
	}
	
	// linear friction
	VectorXd fc_fric = _S_fric * fc;
	int i_linear_friction = 0;
	for(int j=0 ; j<_n_contacts ; j++)
	{
		if(_has_friction_constraints[j])
		{
			Vector3d fc_fric_current = fc_fric.segment<3>(3*i_linear_friction);
			double gi = -fc_fric_current.dot(_R_mu_friction[j]*fc_fric_current);

			if(verbose)
			{
				cout << "contact number : " << j << endl;
				cout << "gi : " << gi << endl;
				// cout << endl;
			}

			if(s + gi <= 0)
			{
				if(verbose)
				{
					cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiPhase1()" << endl;
					cout << endl;
				}
				return numeric_limits<double>::max();
			}
			else
			{
				phi -= log(s + gi);
			}
			i_linear_friction++;
		}
	}

	// tilt
	bool consider_tilt = _contact_patch_dimmensions.size() > 0;
	if(consider_tilt)
	{
		VectorXd fc_tilt = _S_tilt * fc;
		VectorXd r_tilt = _b_tilt - _A_tilt*fc_tilt + s*VectorXd::Ones(_b_tilt.size());

		if(verbose)
		{
			cout << "r_tilt : " << r_tilt.transpose() << endl;
			// cout << endl;
		}

		for(int i=0 ; i<r_tilt.size() ; i++)
		{
			if(r_tilt(i) < 0)
			{
				if(verbose)
				{
					cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiPhase1()" << endl;
					cout << endl;
				}
				return numeric_limits<double>::max();
			}
			else
			{
				phi -= log(r_tilt(i));
			}
		}
	}

	// rotational friction
	bool consider_rfrc = _contact_characteristic_length.size() > 0;
	if(consider_rfrc)
	{
		VectorXd fc_rfrc = _S_rfrc * fc;
		VectorXd r_rfrc = _b_rfrc - _A_rfrc*fc_rfrc + s*VectorXd::Ones(_b_rfrc.size());

		if(verbose)
		{
			cout << "r_rfrc : " << r_rfrc.transpose() << endl;
			// cout << endl;
		}

		for(int i=0 ; i<r_rfrc.size() ; i++)
		{
			if(r_rfrc(i) < 0)
			{
				if(verbose)
				{
					cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiPhase1()" << endl;
					cout << endl;
				}
				return numeric_limits<double>::max();
			}
			else
			{
				phi -= log(r_rfrc(i));
			}
		}
	}

	// force limits
	bool consider_flim = _min_force_limits.size() > 0 || _max_force_limits.size() > 0;
	if(consider_flim)
	{
		VectorXd fc_flim = _S_flim * fc;
		VectorXd r_flim = _b_flim - _A_flim*fc_flim + s*VectorXd::Ones(_b_flim.size());

		if(verbose)
		{
			cout << "r_flim : " << r_flim.transpose() << endl;
			// cout << endl;
		}

		for(int i=0 ; i<r_flim.size() ; i++)
		{
			if(r_flim(i) < 0)
			{
				if(verbose)
				{
					cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiPhase1()" << endl;
					cout << endl;
				}
				return numeric_limits<double>::max();
			}
			else
			{
				phi -= log(r_flim(i));
			}
		}
	}

	// torque limits
	if(_has_torque_constraints)
	{
		VectorXd r_tlim = _b_tlim - _A_tlim*fc + s*VectorXd::Ones(_b_tlim.size());

		if(verbose)
		{
			cout << "r_tlim : " << r_tlim.transpose() << endl;
			// cout << endl;
		}

		for(int i=0 ; i<r_tlim.size() ; i++)
		{
			if(r_tlim(i) < 0)
			{
				if(verbose)
				{
					cout << "premature finish of InternalForceSelectionBarrierMethod::computePhiPhase1()" << endl;
					cout << endl;
				}
				return numeric_limits<double>::max();
			}
			else
			{
				phi -= log(r_tlim(i));
			}
		}
	}

	if(verbose)
	{
		cout << "finished InternalForceSelectionBarrierMethod::computePhiPhase1()" << endl;
		cout << endl;
	}

	return _t_phase1*s + phi;
}

void InternalForceSelectionBarrierMethod::setMetaParametersPhase1(const double t0, const double mu_phase_1)
{
	if(t0 <= 0)
	{
		throw runtime_error("t0 needs to be positive in InternalForceSelectionBarrierMethod::setMetaParametersPhase1(const double t0, const double mu_phase_1)\n");
	}
	if(mu_phase_1 <= 0)
	{
		throw runtime_error("mu needs to be positive in InternalForceSelectionBarrierMethod::setMetaParametersPhase1(const double t0, const double mu_phase_1)\n");
	}

	_t_phase1 = t0;
	_mu_phase1 = mu_phase_1;
}

void InternalForceSelectionBarrierMethod::setBacktrackingLineSearchParameters(const double alpha, const double beta)
{
	if(alpha < 0 || alpha > 0.5)
	{
		throw runtime_error("alpha needs to be strictly between 0 and 0.5 in InternalForceSelectionBarrierMethod::setBacktrackingLineSearchParameters(const double alpha, const double beta)\n");
	}
	if(beta < 0 || beta > 1)
	{
		throw runtime_error("beta needs to be strictly between 0 and 1 in InternalForceSelectionBarrierMethod::setBacktrackingLineSearchParameters(const double alpha, const double beta)\n");
	}

	_alpha_bt_line_search = alpha;
	_beta_bt_line_search = beta;
}

bool InternalForceSelectionBarrierMethod::runPhase1(const double tolerance, const int max_outer_loop_iterations, const int max_inner_loop_iterations)
{
	if(tolerance <= 0)
	{
		throw runtime_error("tolerance parameter needs to be strictly positive in InternalForceSelectionBarrierMethod::runPhase1(const double tolerance, const int max_outer_loop_iterations, const int max_inner_loop_iterations)\n");
	}
	if(!_optimization_ready)
	{
		cout << "call the function prepareOptimization() before you can optimize the forces" << endl;
		return false;
	}

	double phi = computePhi(_fi_current);
	if(phi < numeric_limits<double>::max())
	{
		return true;
	}

	// fing an initial s
	double s = 1000;
	phi = computePhiPhase1(_fi_current, s, _t_phase1);
	while(phi == numeric_limits<double>::max())
	{
		s = s*2;
		phi = computePhiPhase1(_fi_current, s, _t_phase1);
		if(s > 1e30)
		{
			throw runtime_error("abnormal : could not find an initial value of s in InternalForceSelectionBarrierMethod::runPhase1()\n");
		}
	}

	// cout << "starting value for s : " << s << endl;

	// outer loop
	// double outer_gap = _number_of_inequality_constraint_functions/_t_phase1;
	int n_outer_iterations = 0;
	while(s > 0)
	{
		if(n_outer_iterations > max_outer_loop_iterations)
		{
			cout << "phase 1 optimization : excedded the number of outer loop iterations authorized" << endl;
			return false;
		}

		// perform newton method
		int n_inner_iterations = 0;
		double inner_gap = 100.0;
		while(inner_gap > tolerance)
		{
			// cout << n_outer_iterations << "\t" << n_inner_iterations << endl;

			if(n_inner_iterations > max_inner_loop_iterations)
			{
				cout << "phase 1 optimization : excedded the number of inner loop iterations authorized" << endl;
				return false;				
			}

			// compute newton step direction
			VectorXd grad;
			MatrixXd hess;

			computePhiGradHessPhase1(phi, grad, hess, _fi_current, s, _t_phase1);

			VectorXd newton_step_direction = -hess.ldlt().solve(grad);
			double backtracking_line_gap = grad.dot(newton_step_direction);
			inner_gap = backtracking_line_gap*backtracking_line_gap/2.0;

			// compute newton step magnitude with backtracking line search
			int internal_counter = 0;
			double step_magnitude = 1;
			double phi_tmp = computePhiPhase1(_fi_current + step_magnitude*newton_step_direction.head(_fi_dof), s+step_magnitude*newton_step_direction(_fi_dof), _t_phase1);
			while(phi_tmp > phi + _alpha_bt_line_search*step_magnitude*backtracking_line_gap)
			{
				step_magnitude = _beta_bt_line_search*step_magnitude;
				phi_tmp = computePhiPhase1(_fi_current + step_magnitude*newton_step_direction.head(_fi_dof), s+step_magnitude*newton_step_direction(_fi_dof), _t_phase1);
				if(internal_counter > 1000)
				{
					cout << "alpha beta line search failed during phase 1 optimization" << endl;
					return false;
				}
				internal_counter++;
			}

			_fi_current += step_magnitude * newton_step_direction.head(_fi_dof);
			s += step_magnitude * newton_step_direction(_fi_dof);

			n_inner_iterations++;
		
			cout << "inner iteration counter : " << n_inner_iterations << endl;
			cout << "grad (inner iter) :\n" << grad.transpose() << endl;
			cout << "hess (inner iter) :\n" << hess << endl;
			cout << "newton step direction (inner iter) :\n" << newton_step_direction.transpose() << endl;
			cout << "step magnitude : " << step_magnitude << endl;
			cout << "current s (inner iter) :\n" << s << endl;
			cout << "current fi (inner iter) :\n" << _fi_current.transpose() << endl;
			cout << "current expected fc (inner iter) :\n" << _fc_expected.transpose() << endl;
			cout << "inner gap (inner iter) : " << inner_gap << endl;
			cout << endl << endl << endl;

		}

		_t_phase1 *= _mu_phase1;
		// outer_gap = _number_of_inequality_constraint_functions/_t_phase1;


		cout << "finished outer iteration " << n_outer_iterations << " in " << n_inner_iterations << " inner iterations." << endl;
		cout << "inner final gap : " << inner_gap << endl;
		cout << "curent t : " << _t_phase1 << endl;
		// cout << "current outer gap : " << outer_gap << endl;
		cout << "current fi :\n" << _fi_current.transpose() << endl;
		cout << "current s : " << s << endl;
		cout << endl; 

		n_outer_iterations++;
	}

	return true;
}

bool InternalForceSelectionBarrierMethod::computeFi(const double tolerance, const int max_iterations)
{
	if(tolerance <= 0)
	{
		throw runtime_error("tolerance parameter needs to be strictly positive in InternalForceSelectionBarrierMethod::computeFi(const double tolerance, const int max_iterations)\n");
	}
	if(!_optimization_ready)
	{
		cout << "call the function prepareOptimization() before you can optimize the forces" << endl;
		return false;
	}

	double phi = computePhi(_fi_current);
	if(phi == numeric_limits<double>::max())
	{
		cout << "initial guess not feasible. Run phase 1 to find a suitable initial guess" << endl;
		return false;
	}

	// perform newton method
	int n_inner_iterations = 0;
	double inner_gap = 100.0;
	while(inner_gap > tolerance)
	{
		if(n_inner_iterations > max_iterations)
		{
			cout << "Fi optimization : excedded the number of inner loop iterations authorized" << endl;
			return false;				
		}

		// compute newton step direction
		VectorXd grad;
		MatrixXd hess;

		computePhiGradHess(phi, grad, hess, _fi_current);

		VectorXd newton_step_direction = -hess.ldlt().solve(grad);
		double backtracking_line_gap = grad.dot(newton_step_direction);
		inner_gap = backtracking_line_gap*backtracking_line_gap/2.0;

		// compute newton step magnitude with backtracking line search
		int internal_counter = 0;
		double step_magnitude = 1;
		double phi_tmp = computePhi(_fi_current + step_magnitude*newton_step_direction);
		while(phi_tmp > phi + _alpha_bt_line_search*step_magnitude*backtracking_line_gap + tolerance)
		{
			step_magnitude = _beta_bt_line_search*step_magnitude;
			phi_tmp = computePhi(_fi_current + step_magnitude*newton_step_direction);
			if(internal_counter > 1000)
			{
				cout << "alpha beta line search failed during optimization" << endl;
				return false;
			}
			internal_counter++;
		}

		_fi_current += step_magnitude * newton_step_direction;

		// cout << "step magnitude : " << step_magnitude << endl;

		n_inner_iterations++;
	}

	if(n_inner_iterations > 1)
	{
		cout << "optimization finished in " << n_inner_iterations << " iterations" << endl << endl;
	}
	// cout << "resulting Fc :\n" << (_W_bar_extended*_fr_fac + _N*_fi_current).transpose() << endl;
	// cout << endl;

	return true;

}

VectorXd InternalForceSelectionBarrierMethod::getFi()
{
	return _fi_current;
}

} // namesapce Sai2Primitives
