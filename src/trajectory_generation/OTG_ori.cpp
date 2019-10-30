/**
 * OTG_ori.cpp
 *
 *	A wrapper to use the Reflexxes library (type II or IV) with Eigen library
 *	specifically to work for orientation using rotation matrices
 *
 * Author: Mikael Jorda
 * Created: October 2018
 */

#include "OTG_ori.h"
#include <stdexcept>

namespace Sai2Primitives
{

OTG_ori::OTG_ori(const Eigen::Matrix3d& initial_orientation, const double loop_time)
{
	_loop_time = loop_time;

    _IP  = new RMLPositionInputParameters(3);
    _OP  = new RMLPositionOutputParameters(3);
	_RML = new ReflexxesAPI(3, _loop_time);

    setGoalPositionAndVelocity(initial_orientation, initial_orientation, Eigen::Vector3d::Zero());
    for(int i=0 ; i<3 ; i++)
    {
    	_IP->SelectionVector->VecData[i] = true;
		_OP->NewPositionVector->VecData[i] = 0;
    }
}

OTG_ori::~OTG_ori()
{
	delete _RML;
	delete _IP;
	delete _OP;
	_RML = NULL;
	_IP = NULL;
	_OP = NULL;
}

void OTG_ori::reInitialize(const Eigen::Matrix3d& initial_orientation)
{
    setGoalPositionAndVelocity(initial_orientation, initial_orientation, Eigen::Vector3d::Zero());

    for(int i=0 ; i<3 ; i++)
    {
		_OP->NewPositionVector->VecData[i] = 0;
		_OP->NewVelocityVector->VecData[i] = 0;
		_OP->NewAccelerationVector->VecData[i] = 0;
    }	
}

void OTG_ori::setMaxVelocity(const Eigen::Vector3d max_velocity)
{
	if(max_velocity.size() != 3)
	{
		throw std::invalid_argument("size of input max velocity vector does not match task size in OTG_ori::setMaxVelocity\n");
	}
	if(max_velocity.minCoeff() <= 0)
	{
		throw std::invalid_argument("max velocity set to 0 or negative value in some directions in OTG_ori::setMaxVelocity\n");
	}

	for(int i=0 ; i<3 ; i++)
	{
	    _IP->MaxVelocityVector->VecData[i] = max_velocity(i);
	}
}

void OTG_ori::setMaxVelocity(const double max_velocity)
{
	setMaxVelocity(max_velocity * Eigen::VectorXd::Ones(3));
}

void OTG_ori::setMaxAcceleration(const Eigen::Vector3d max_acceleration)
{
	if(max_acceleration.size() != 3)
	{
		throw std::invalid_argument("size of input max acceleration vector does not match task size in OTG_ori::setMaxAcceleration\n");
	}
	if(max_acceleration.minCoeff() <= 0)
	{
		throw std::invalid_argument("max acceleration set to 0 or negative value in some directions in OTG_ori::setMaxAcceleration\n");
	}

	for(int i=0 ; i<3 ; i++)
	{
	    _IP->MaxAccelerationVector->VecData[i] = max_acceleration(i);
	}
}

void OTG_ori::setMaxAcceleration(const double max_acceleration)
{
	setMaxAcceleration(max_acceleration * Eigen::VectorXd::Ones(3));
}

void OTG_ori::setMaxJerk(const Eigen::Vector3d max_jerk)
{
	if(max_jerk.size() != 3)
	{
		throw std::invalid_argument("size of input max jerk vector does not match task size in OTG_ori::setMaxJerk\n");
	}
	if(max_jerk.minCoeff() <= 0)
	{
		throw std::invalid_argument("max jerk set to 0 or negative value in some directions in OTG_ori::setMaxJerk\n");
	}

	for(int i=0 ; i<3 ; i++)
	{
	    _IP->MaxJerkVector->VecData[i] = max_jerk(i);
	}
}

void OTG_ori::setMaxJerk(const double max_jerk)
{
	setMaxJerk(max_jerk * Eigen::VectorXd::Ones(3));
}

void OTG_ori::setGoalPositionAndVelocity(const Eigen::Matrix3d goal_orientation, const Eigen::Matrix3d current_orientation, const Eigen::Vector3d goal_velocity)
{
	if((goal_orientation.transpose() * goal_orientation - Eigen::Matrix3d::Identity()).norm() > 1e-6)
	{
		throw std::invalid_argument("goal orientation is not a valid rotation matrix OTG_ori::setGoalPosition\n");
	}
	if(abs(goal_orientation.determinant() - 1) > 1e-6)
	{
		throw std::invalid_argument("goal orientation is not a valid rotation matrix OTG_ori::setGoalPosition\n");
	}
	if((current_orientation.transpose() * current_orientation - Eigen::Matrix3d::Identity()).norm() > 1e-6)
	{
		throw std::invalid_argument("current orientation is not a valid rotation matrix OTG_ori::setGoalPosition\n");
	}
	if(abs(current_orientation.determinant() - 1) > 1e-6)
	{
		throw std::invalid_argument("current orientation is not a valid rotation matrix OTG_ori::setGoalPosition\n");
	}
	if( (_goal_orientation != goal_orientation) || (_goal_angular_velocity_in_base_frame != goal_velocity) )
	{
		_goal_reached = false;
		Eigen::Matrix3d R_new_to_previous_initial_frame = current_orientation.transpose()*_initial_orientation;
		_initial_orientation = current_orientation;
		_goal_orientation = goal_orientation;
		_goal_angular_velocity_in_base_frame = goal_velocity;

		Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
		Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
		for(int i = 0 ; i<3 ; i++)
		{
			velocity(i) = _OP->NewVelocityVector->VecData[i];
			acceleration(i) = _OP->NewAccelerationVector->VecData[i];
		}
		velocity = R_new_to_previous_initial_frame * velocity;
		acceleration = R_new_to_previous_initial_frame * acceleration;

		Eigen::Matrix3d goal_rot_relative_to_current = _initial_orientation.transpose()*goal_orientation;
		Eigen::AngleAxisd rot_rel_aa = Eigen::AngleAxisd(goal_rot_relative_to_current);
		Eigen::Vector3d rot_representation = rot_rel_aa.angle() * rot_rel_aa.axis();
		Eigen::Vector3d goal_velocity_in_current_frame = _initial_orientation.transpose()*goal_velocity;

		for(int i=0 ; i<3 ; i++)
		{
			_OP->NewPositionVector->VecData[i] = 0;
			_OP->NewVelocityVector->VecData[i] = velocity(i);
			_OP->NewAccelerationVector->VecData[i] = acceleration(i);
			_IP->TargetPositionVector->VecData[i] = rot_representation(i);
			_IP->TargetVelocityVector->VecData[i] = goal_velocity_in_current_frame(i);
		}
	}

}

void OTG_ori::computeNextState(Eigen::Matrix3d& next_orientation, Eigen::Vector3d& next_angular_velocity, Eigen::Vector3d& next_angular_acceleration)
{
	Eigen::Vector3d next_ori_representation = Eigen::Vector3d::Zero();

	for(int i=0 ; i<3 ; i++)
	{
		_IP->CurrentPositionVector->VecData[i] = _OP->NewPositionVector->VecData[i];
	    _IP->CurrentVelocityVector->VecData[i] = _OP->NewVelocityVector->VecData[i];
	    _IP->CurrentAccelerationVector->VecData[i] = _OP->NewAccelerationVector->VecData[i];
		next_ori_representation(i) = _IP->TargetPositionVector->VecData[i];
		next_angular_velocity(i) = 0;
		next_angular_acceleration(i) = 0;
	}

	if(!_goal_reached)
	{
		_ResultValue = _RML->RMLPosition(*_IP, _OP, _Flags);
        if (_ResultValue < 0)
        {
        	if(_ResultValue == -102)
        	{
        		printf("trajectory generation : phase synchronisation not possible.\n");
        	}
        	else
        	{
	        	printf("An error occurred (%d) in OTG_ori::computeNextState.\n", _ResultValue );
	            throw std::runtime_error("error in computing next state in OTG_ori::computeNextState.\n");
        	}
        }

		for(int i=0 ; i<3 ; i++)
		{
			next_ori_representation(i) = _OP->NewPositionVector->VecData[i];
			next_angular_velocity(i) = _OP->NewVelocityVector->VecData[i];
			next_angular_acceleration(i) = _OP->NewAccelerationVector->VecData[i];
		}
	}

	if(next_ori_representation.norm() < 1e-3)
	{
		next_orientation.setIdentity();
	}
	else
	{
		next_orientation = Eigen::AngleAxisd(next_ori_representation.norm(), next_ori_representation.normalized()).toRotationMatrix();
	}

	next_orientation = _initial_orientation*next_orientation;
	next_angular_velocity = _initial_orientation*next_angular_velocity;
	next_angular_acceleration = _initial_orientation*next_angular_acceleration;

	_goal_reached = (_ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
	
}

bool OTG_ori::goalReached()
{
	return _goal_reached;
}

} /* namespace Sai2Primitives */