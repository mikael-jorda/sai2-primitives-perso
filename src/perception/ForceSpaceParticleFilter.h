

#ifndef FORCE_SPACE_PARTICLE_FILTER_WEIGHT_MEM_H_
#define FORCE_SPACE_PARTICLE_FILTER_WEIGHT_MEM_H_

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

	ForceSpaceParticleFilter(const int n_particles);
	~ForceSpaceParticleFilter(){}

	void update(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured);

	Matrix3d getSigmaForce();


	vector<pair<Vector3d, double>> motionUpdateAndWeighting(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured);

	void resamplingLowVariance(vector<pair<Vector3d, double>> weighted_particles);
	void computePCA();

	double sampleNormalDistribution(const double mean, const double std);
	double sampleUniformDistribution(const double min, const double max);

	double wf(const Vector3d particle, const Vector3d force_measured, const double fl, const double fh);
	double wv(const Vector3d particle, const Vector3d velocity_measured, const double vl, const double vh);


	int _n_particles;
	vector<Vector3d> _particles;
	vector<pair<Vector3d,double>> _particles_with_weight;

	double _mean_scatter;
	double _std_scatter;

	double _coeff_friction;

	int _force_space_dimension;

	Vector3d _force_axis;
	Vector3d _motion_axis;

	double _F_low, _F_high, _v_high, _v_low;
	double _F_low_add, _F_high_add, _v_high_add, _v_low_add;

	Matrix3d _eigenvectors;
	Vector3d _eigenvalues;

	double _alpha_add, _alpha_remove;

};

} // namesapce Sai2Primitives

/* FORCE_SPACE_PARTICLE_FILTER_WEIGHT_MEM_H_ */
#endif
