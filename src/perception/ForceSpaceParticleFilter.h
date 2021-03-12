/*
 * ForceSpaceParticleFilter.h
 *
 *      Estimates contact constraint directions with the Force Space Particle Filter as described in
 *		Jorda M. Robust Robotic Manipulation for Effective Multi-contact and Safe Physical Interactions (Chapter 6)
 *		USA Stanford 2021
 *		
 *      Author: Mikael Jorda
 */

#ifndef FORCE_SPACE_PARTICLE_FILTER_H_
#define FORCE_SPACE_PARTICLE_FILTER_H_

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <random>

using namespace Eigen;
using namespace std;

namespace Sai2Primitives
{

class ForceSpaceParticleFilter
{
public:

	/**
	 * @brief      Constructor
	 *
	 * @param[in]  n_particles  number of particles for the particle filter
	 */
	ForceSpaceParticleFilter(const int n_particles);
	/**
	 * @brief      Destructor
	 */
	~ForceSpaceParticleFilter(){}

	/**
	 * @brief      Computes the full particle filter update step (particles
	 *             updates, particles resampling and contact direction estimate)
	 *             and computes the most recent estimate of sigma_force. All the
	 *             inputs are expressed in world frame.
	 *
	 * @param[in]  motion_control     Control input in motion space
	 * @param[in]  force_control      Control input in force space
	 * @param[in]  velocity_measured  Measured velocity
	 * @param[in]  force_measured     Sensed force
	 */
	void update(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured);

	/**
	 * @brief      Returns the estimate of the force space selection matric
	 *
	 * @return     Force space selection matrix
	 */
	Matrix3d getSigmaForce();



	/////////// Internal use functions /////////////////////

	vector<pair<Vector3d, double>> motionUpdateAndWeighting(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured);

	void resamplingLowVariance(vector<pair<Vector3d, double>> weighted_particles);
	void computePCA();

	double sampleNormalDistribution(const double mean, const double std);
	double sampleUniformDistribution(const double min, const double max);

	double wf(const Vector3d particle, const Vector3d force_measured, const double fl, const double fh);
	double wv(const Vector3d particle, const Vector3d velocity_measured, const double vl, const double vh);


	//////////// Member variables ///////////////
	// Parameters the user can set
	double _mean_scatter;             // to bias the random motion of the particles in a direction. 0 by default
	double _std_scatter;              // stadard deviation of the uniform random motion of the particles in the motion step default is 0.025

	double _F_low, _F_high, _v_high, _v_low;      // weight function parameters for the resampling step
	double _F_low_add, _F_high_add, _v_high_add, _v_low_add;   // weight function parameters for adding particles in the motion step

	double _alpha_add, _alpha_remove;    // hysteresis parameters for finding constraint directions from the SVD

	/////////////// Internal use only ////////////////////

	int _n_particles;
	vector<Vector3d> _particles;
	vector<pair<Vector3d,double>> _particles_with_weight;

	double _coeff_friction;

	int _force_space_dimension;

	Vector3d _force_axis;
	Vector3d _motion_axis;

	Matrix3d _eigenvectors;
	Vector3d _eigenvalues;

};

} // namesapce Sai2Primitives

/* FORCE_SPACE_PARTICLE_FILTER_H_ */
#endif
