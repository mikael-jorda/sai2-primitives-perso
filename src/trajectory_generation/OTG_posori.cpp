/**
 * OTG_posori.cpp
 *
 *	A wrapper to use the Reflexxes library (type II or IV) with Eigen library
 *	specifically to work for 6DOF position and orientation
 *
 * Author: Mikael Jorda
 * Created: February 2019
 */

#include "OTG_posori.h"
#include <stdexcept>

namespace Sai2Primitives
{

OTG_posori::OTG_posori(const Eigen::VectorXd& initial_position, 
		const Eigen::Matrix3d& initial_orientation, 
		const double loop_time)
{
	_loop_time = loop_time;

    _IP  = new RMLPositionInputParameters(6);
    _OP  = new RMLPositionOutputParameters(6);
	_RML = new ReflexxesAPI(6, _loop_time);

    for(int i=0 ; i<3 ; i++)
    {
    	_IP->SelectionVector->VecData[i] = true;
    	_IP->SelectionVector->VecData[i+3] = true;
    }
    reInitialize(initial_position, initial_orientation);
}

OTG_posori::~OTG_posori()
{
	delete _RML;
	delete _IP;
	delete _OP;
	_RML = NULL;
	_IP = NULL;
	_OP = NULL;
}

void OTG_posori::reInitialize(const Eigen::VectorXd& initial_position, const Eigen::Matrix3d& initial_orientation)
{
    setGoalPositionAndLinearVelocity(initial_position, Eigen::Vector3d::Zero());
    setGoalOrientationAndAngularVelocity(initial_orientation, initial_orientation, Eigen::Vector3d::Zero());

    for(int i=0 ; i<3 ; i++)
    {
		_OP->NewPositionVector->VecData[i] = initial_position(i);
		_OP->NewVelocityVector->VecData[i] = 0;
		_OP->NewAccelerationVector->VecData[i] = 0;

		_OP->NewPositionVector->VecData[i+3] = 0;
		_OP->NewVelocityVector->VecData[i+3] = 0;
		_OP->NewAccelerationVector->VecData[i+3] = 0;
    }	
}

void OTG_posori::setMaxLinearVelocity(const Eigen::Vector3d max_linear_velocity)
{
	if(max_linear_velocity.minCoeff() <= 0)
	{
		throw std::invalid_argument("max velocity set to 0 or negative value in some directions in OTG_posori::setMaxLinearVelocity\n");
	}

	for(int i=0 ; i<3 ; i++)
	{
	    _IP->MaxVelocityVector->VecData[i] = max_linear_velocity(i);
	}
}

void OTG_posori::setMaxLinearVelocity(const double max_linear_velocity)
{
	setMaxLinearVelocity(max_linear_velocity * Eigen::VectorXd::Ones(3));
}

void OTG_posori::setMaxLinearAcceleration(const Eigen::Vector3d max_linear_acceleration)
{
	if(max_linear_acceleration.minCoeff() <= 0)
	{
		throw std::invalid_argument("max acceleration set to 0 or negative value in some directions in OTG_posori::setMaxLinearAcceleration\n");
	}

	for(int i=0 ; i<3 ; i++)
	{
	    _IP->MaxAccelerationVector->VecData[i] = max_linear_acceleration(i);
	}
}

void OTG_posori::setMaxLinearAcceleration(const double max_linear_acceleration)
{
	setMaxLinearAcceleration(max_linear_acceleration * Eigen::VectorXd::Ones(3));
}

void OTG_posori::setMaxLinearJerk(const Eigen::Vector3d max_linear_jerk)
{
	if(max_linear_jerk.minCoeff() <= 0)
	{
		throw std::invalid_argument("max jerk set to 0 or negative value in some directions in OTG_posori::setMaxLinearJerk\n");
	}

	for(int i=0 ; i<3 ; i++)
	{
	    _IP->MaxJerkVector->VecData[i] = max_linear_jerk(i);
	}
}

void OTG_posori::setMaxLinearJerk(const double max_linear_jerk)
{
	setMaxLinearJerk(max_linear_jerk * Eigen::VectorXd::Ones(3));
}

void OTG_posori::setMaxAngularVelocity(const Eigen::Vector3d max_velocity)
{
	if(max_velocity.minCoeff() <= 0)
	{
		throw std::invalid_argument("max velocity set to 0 or negative value in some directions in OTG_posori::setMaxAngularVelocity\n");
	}

	for(int i=0 ; i<3 ; i++)
	{
	    _IP->MaxVelocityVector->VecData[i+3] = max_velocity(i);
	}
}

void OTG_posori::setMaxAngularVelocity(const double max_angular_velocity)
{
	setMaxAngularVelocity(max_angular_velocity * Eigen::VectorXd::Ones(3));
}

void OTG_posori::setMaxAngularAcceleration(const Eigen::Vector3d max_angular_acceleration)
{
	if(max_angular_acceleration.minCoeff() <= 0)
	{
		throw std::invalid_argument("max acceleration set to 0 or negative value in some directions in OTG_posori::setMaxAngularAcceleration\n");
	}

	for(int i=0 ; i<3 ; i++)
	{
	    _IP->MaxAccelerationVector->VecData[i+3] = max_angular_acceleration(i);
	}
}

void OTG_posori::setMaxAngularAcceleration(const double max_angular_acceleration)
{
	setMaxAngularAcceleration(max_angular_acceleration * Eigen::VectorXd::Ones(3));
}

void OTG_posori::setMaxAngularJerk(const Eigen::Vector3d max_angular_jerk)
{
	if(max_angular_jerk.minCoeff() <= 0)
	{
		throw std::invalid_argument("max jerk set to 0 or negative value in some directions in OTG_posori::setMaxAngularJerk\n");
	}

	for(int i=0 ; i<3 ; i++)
	{
	    _IP->MaxJerkVector->VecData[i+3] = max_angular_jerk(i);
	}
}

void OTG_posori::setMaxAngularJerk(const double max_angular_jerk)
{
	setMaxAngularJerk(max_angular_jerk * Eigen::VectorXd::Ones(3));
}

void OTG_posori::setGoalPositionAndLinearVelocity(const Eigen::Vector3d goal_position, const Eigen::Vector3d goal_linear_velocity)
{
	for(int i=0 ; i<3 ; i++)
	{
		if( (_IP->TargetPositionVector->VecData[i] != goal_position(i)) || (_IP->TargetVelocityVector->VecData[i] != goal_linear_velocity(i)) )
		{
			_IP->TargetPositionVector->VecData[i] = goal_position(i);
			_IP->TargetVelocityVector->VecData[i] = goal_linear_velocity(i);
			_goal_reached = false;
		}
	}
}

void OTG_posori::setGoalOrientationAndAngularVelocity(const Eigen::Matrix3d goal_orientation, const Eigen::Matrix3d current_orientation, const Eigen::Vector3d goal_angular_velocity)
{
	if((goal_orientation.transpose() * goal_orientation - Eigen::Matrix3d::Identity()).norm() > 1e-3)
	{
		throw std::invalid_argument("goal orientation is not a valid rotation matrix OTG_posori::setGoalOrientationAndAngularVelocity\n");
	}
	if(abs(goal_orientation.determinant() - 1) > 1e-3)
	{
		throw std::invalid_argument("goal orientation is not a valid rotation matrix OTG_posori::setGoalOrientationAndAngularVelocity\n");
	}
	if((current_orientation.transpose() * current_orientation - Eigen::Matrix3d::Identity()).norm() > 1e-3)
	{
		throw std::invalid_argument("current orientation is not a valid rotation matrix OTG_posori::setGoalOrientationAndAngularVelocity\n");
	}
	if(abs(current_orientation.determinant() - 1) > 1e-3)
	{
		throw std::invalid_argument("current orientation is not a valid rotation matrix OTG_posori::setGoalOrientationAndAngularVelocity\n");
	}
	if( (_goal_orientation != goal_orientation) || (_goal_angular_velocity_in_base_frame != goal_angular_velocity) )
	{
		_goal_reached = false;
		Eigen::Matrix3d R_new_to_previous_initial_frame = current_orientation.transpose()*_initial_orientation;
		_initial_orientation = current_orientation;
		_goal_orientation = goal_orientation;
		_goal_angular_velocity_in_base_frame = goal_angular_velocity;

		Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
		Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
		for(int i = 0 ; i<3 ; i++)
		{
			velocity(i) = _OP->NewVelocityVector->VecData[i+3];
			acceleration(i) = _OP->NewAccelerationVector->VecData[i+3];
		}
		velocity = R_new_to_previous_initial_frame * velocity;
		acceleration = R_new_to_previous_initial_frame * acceleration;

		Eigen::Matrix3d goal_rot_relative_to_current = _initial_orientation.transpose()*goal_orientation;
		Eigen::AngleAxisd rot_rel_aa = Eigen::AngleAxisd(goal_rot_relative_to_current);
		Eigen::Vector3d rot_representation = rot_rel_aa.angle() * rot_rel_aa.axis();
		Eigen::Vector3d goal_velocity_in_current_frame = _initial_orientation.transpose()*goal_angular_velocity;

		for(int i=0 ; i<3 ; i++)
		{
			_OP->NewPositionVector->VecData[i+3] = 0;
			_OP->NewVelocityVector->VecData[i+3] = velocity(i);
			_OP->NewAccelerationVector->VecData[i+3] = acceleration(i);
			_IP->TargetPositionVector->VecData[i+3] = rot_representation(i);
			_IP->TargetVelocityVector->VecData[i+3] = goal_velocity_in_current_frame(i);
		}
	}

}

void OTG_posori::computeNextState(Eigen::Vector3d& next_position, Eigen::Vector3d& next_linear_velocity,
			Eigen::Matrix3d& next_orientation, Eigen::Vector3d& next_angular_velocity)
{
	Eigen::Vector3d next_ori_representation = Eigen::Vector3d::Zero();

	for(int i=0 ; i<3 ; i++)
	{
		_IP->CurrentPositionVector->VecData[i] = _OP->NewPositionVector->VecData[i];
	    _IP->CurrentVelocityVector->VecData[i] = _OP->NewVelocityVector->VecData[i];
	    _IP->CurrentAccelerationVector->VecData[i] = _OP->NewAccelerationVector->VecData[i];
		next_position(i) = _IP->CurrentPositionVector->VecData[i];
		next_linear_velocity(i) = 0;

		_IP->CurrentPositionVector->VecData[i+3] = _OP->NewPositionVector->VecData[i+3];
	    _IP->CurrentVelocityVector->VecData[i+3] = _OP->NewVelocityVector->VecData[i+3];
	    _IP->CurrentAccelerationVector->VecData[i+3] = _OP->NewAccelerationVector->VecData[i+3];
		next_ori_representation(i) = _IP->CurrentPositionVector->VecData[i+3];
		next_angular_velocity(i) = 0;
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
	        	printf("An error occurred (%d) in OTG_posori::computeNextState.\nTrajectory Stopped\n", _ResultValue );
	            return;
	            // throw std::runtime_error("error in computing next state in OTG_posori::computeNextState.\n");
        	}
        }

		for(int i=0 ; i<3 ; i++)
		{
			next_position(i) = _OP->NewPositionVector->VecData[i];
			next_linear_velocity(i) = _OP->NewVelocityVector->VecData[i];
			next_ori_representation(i) = _OP->NewPositionVector->VecData[i+3];
			next_angular_velocity(i) = _OP->NewVelocityVector->VecData[i+3];
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

	_goal_reached = (_ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
	
}

bool OTG_posori::goalReached()
{
	return _goal_reached;
}

} /* namespace Sai2Primitives */