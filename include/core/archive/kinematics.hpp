#ifndef KIN_HPP
#define KIN_HPP
#include <Eigen/Dense>
#include <vector>
#include "core/trajectory_generation.hpp"

//struct TrajectoryKinematics{
//	Eigen::MatrixXd deviation = Eigen::MatrixXd::Zero(8000, 3);
//	Eigen::MatrixXd position  = Eigen::MatrixXd::Zero(8000,3);
//	Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(8000, 3);
//	Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(8000, 3);
//	Eigen::MatrixXd jerk = Eigen::MatrixXd::Zero(8000, 3);
//	Eigen::Vector3d initPosition = Eigen::Vector3d::Zero();
//	Eigen::Vector3d initVelocity = Eigen::Vector3d::Zero();
//	Eigen::Vector3d initAcceleration = Eigen::Vector3d::Zero();
//	double dt = 0.00025;
//};
struct Constraints {
	Eigen::Vector3d min_pos = Eigen::Vector3d(-10.0,-10.0,-610.0);
	Eigen::Vector3d max_pos = Eigen::Vector3d(1067.0,610.0,0);
	Eigen::Vector3d max_vel = Eigen::Vector3d(533, 533, 533);
	Eigen::Vector3d max_acc = Eigen::Vector3d(2450, 2450, 2450);
	Eigen::Vector3d max_jerk = Eigen::Vector3d(12250, 12250, 12250);
	double safety_margin = 0.1; // Safety margin for constraints
};
#endif // !KIN_HPP
enum class NoiseType {
	GAUSSIAN, FILTERED_UNIFORM, SINUSOIDAL
};
class KinematicsCalculator {
public:
	static TrajectoryKinematics computeKinematicDerivatives(const Eigen::MatrixXd& position, double dt);
	static void computeKinematicDerivatives(TrajectoryKinematics& trajectory);
	static void reconstructTrajectoryFromJerk(TrajectoryKinematics& trajectory);
private:
static Eigen::MatrixXd differentiateSignal(const Eigen::MatrixXd& signal, double dt);
static Eigen::MatrixXd integrateSignal(const Eigen::MatrixXd& signal, double dt, const Eigen::Vector3d& initCondition);
friend class ConstrainedDisturbanceGenerator;
};
class ConstrainedDisturbanceGenerator {
public:
	ConstrainedDisturbanceGenerator(Constraints constraints, unsigned int seed = 42);
	TrajectoryKinematics generateDisturbance(const TrajectoryKinematics& inputTrajectory,NoiseType noiseType, double param1 = 0.0, double param2 = 0.0,double param3 = 0.0,double param4 = 0.0);
	void cleanTrajectory(TrajectoryKinematics& trajectory);
	//TrajectoryKinematics cleanTrajectory_v2(const TrajectoryKinematics& trajectory);
private:
	Constraints constraints;
	unsigned int seed;
	void gaussianNoise(const TrajectoryKinematics& inputTrajectory,TrajectoryKinematics& trajectory,double mean,double stdev);
	void filteredUniformNoise(const TrajectoryKinematics& inputTrajectory, TrajectoryKinematics& trajectory, double max_amp,double alpha);
	void sinusoidalNoise(const TrajectoryKinematics& inputTrajectory, TrajectoryKinematics& trajectory, double max_amp, double min_freq = 1.0, double max_freq = 2000.0,double maxSamples = 10);

};