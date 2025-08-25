#include "core/kinematics.hpp"
#include "core/trajectory_generation.hpp"
#include <random>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

TrajectoryKinematics KinematicsCalculator::computeKinematicDerivatives(const Eigen::MatrixXd& position, double dt) {
	TrajectoryKinematics kinematics;
	kinematics.dt = dt;
	kinematics.position = position;
	kinematics.initPosition = position.row(0);
	kinematics.velocity = differentiateSignal(kinematics.position, dt);
	kinematics.initVelocity = kinematics.velocity.row(0);
	kinematics.acceleration = differentiateSignal(kinematics.velocity, dt);
	kinematics.initAcceleration = kinematics.acceleration.row(0);
	kinematics.jerk = differentiateSignal(kinematics.acceleration, dt);
	return kinematics;
}

void KinematicsCalculator::computeKinematicDerivatives(TrajectoryKinematics& trajectory) {
	trajectory.velocity = differentiateSignal(trajectory.position, trajectory.dt);
	trajectory.acceleration = differentiateSignal(trajectory.velocity, trajectory.dt);
	trajectory.jerk = differentiateSignal(trajectory.acceleration, trajectory.dt);
}

void KinematicsCalculator::reconstructTrajectoryFromJerk(TrajectoryKinematics& trajectory) {
	trajectory.acceleration = integrateSignal(trajectory.jerk, trajectory.dt, trajectory.initAcceleration);
	trajectory.velocity = integrateSignal(trajectory.acceleration, trajectory.dt, trajectory.initVelocity);
	trajectory.position = integrateSignal(trajectory.velocity, trajectory.dt, trajectory.initPosition);
}

Eigen::MatrixXd KinematicsCalculator::differentiateSignal(
	const Eigen::MatrixXd& signal, double dt)
{
	int rows = signal.rows();
	int cols = signal.cols();
	Eigen::MatrixXd derivative(rows, cols);

	for (int col = 0; col < cols; ++col) {
		// Forward difference at the first element
		derivative(0, col) = (signal(1, col) - signal(0, col)) / dt;

		// Central difference
		for (int row = 1; row < rows - 1; ++row) {
			derivative(row, col) = (signal(row + 1, col) - signal(row - 1, col)) / (2.0 * dt);
		}

		// Backward difference at the last element
		derivative(rows - 1, col) = (signal(rows - 1, col) - signal(rows - 2, col)) / dt;
	}

	return derivative;
}

Eigen::MatrixXd KinematicsCalculator::integrateSignal(
	const Eigen::MatrixXd& signal, double dt, const Eigen::Vector3d& initCondition)
{
	int rows = signal.rows();
	int cols = signal.cols();
	Eigen::MatrixXd integral(rows, cols);

	// Set initial condition
	for (int col = 0; col < cols; ++col) {
		integral(0, col) = initCondition(col);
	}

	// Integrate using cumulative sum
	for (int col = 0; col < cols; ++col) {
		for (int row = 1; row < rows; ++row) {
			integral(row, col) = integral(row - 1, col) + signal(row, col) * dt;
		}
	}
	return integral;
}

ConstrainedDisturbanceGenerator::ConstrainedDisturbanceGenerator(Constraints constraints, unsigned int seed):
	constraints(constraints),
	seed(seed) {
}
TrajectoryKinematics ConstrainedDisturbanceGenerator::generateDisturbance(const TrajectoryKinematics& inputTrajectory, NoiseType noiseType, double param1, double param2, double param3, double param4)
{
	TrajectoryKinematics trajectory = inputTrajectory;
	//cleanTrajectory(trajectory);
	//switch (noiseType) {
	//case NoiseType::GAUSSIAN:
	//	gaussianNoise(cleanedTrajectory, trajectory, param1, param2);  // mean, stdev
	//	break;
	//case NoiseType::FILTERED_UNIFORM:
	//	filteredUniformNoise(cleanedTrajectory, trajectory, param1, param2);  // max_amp, alpha
	//	break;
	//case NoiseType::SINUSOIDAL:
	//	sinusoidalNoise(cleanedTrajectory,trajectory, param1, param2, param3,param4); // max_amp, min_freq, max_freq
	//	break;
	//}
	KinematicsCalculator::computeKinematicDerivatives(trajectory);
	//cleanTrajectory(trajectory);
	trajectory.deviation = trajectory.position - inputTrajectory.position;
	return trajectory;
}

void ConstrainedDisturbanceGenerator::gaussianNoise(const TrajectoryKinematics& inputTrajectory, TrajectoryKinematics& trajectory, double mean, double stdev) {
	std::default_random_engine generator(seed);
	std::normal_distribution<double> distribution(mean, stdev);
	for (int i = 0; i < trajectory.position.rows(); ++i) {
		for (int j = 0; j < trajectory.position.cols(); ++j) {
			trajectory.position(i, j) = inputTrajectory.position(i,j) + distribution(generator);
		}
	}
}
void ConstrainedDisturbanceGenerator::filteredUniformNoise(const TrajectoryKinematics& inputTrajectory,TrajectoryKinematics& trajectory, double max_amp, double alpha) {
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distribution(-max_amp, max_amp);
	int rows = trajectory.position.rows();
	int cols = trajectory.position.cols();
	double last_value = 0.0;
	for (int j = 0; j < cols; ++j) {
		double last_value = 0.0;  // Reset for each dimension
		for (int i = 0; i < rows; ++i) {
			double noise = distribution(generator);
			trajectory.position(i, j) = inputTrajectory.position(i, j) + alpha * last_value + (1.0 - alpha) * noise;
			last_value = trajectory.position(i, j);
		}
	}
}

void ConstrainedDisturbanceGenerator::sinusoidalNoise(const TrajectoryKinematics& inputTrajectory,TrajectoryKinematics& trajectory, double max_amp, double min_freq, double max_freq, double maxSamples) {
	double allowableJerk = (1 - constraints.safety_margin) * constraints.max_jerk.minCoeff();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> ampDist(0.0, std::min(max_amp,allowableJerk));
	std::uniform_real_distribution<double> freqDist(min_freq, max_freq);
	std::uniform_real_distribution<double> phaseDist(0.0, 2 * M_PI);
	std::uniform_int_distribution<int> sampleDist(1, maxSamples);
	int s = sampleDist(generator);
	trajectory.position = inputTrajectory.position;
	int rows = trajectory.position.rows();
	int cols = trajectory.position.cols();

	for (int n = 0; n < s; ++n) {
		for (int col = 0; col < cols; ++col) {
			double freq = freqDist(generator);
			double phase = phaseDist(generator);
			double amp = ampDist(generator);
			for (int row = 0; row < rows; ++row) {
				double t = row * trajectory.dt;  // time at this sample
				trajectory.position(row, col) += amp * std::sin(2 * M_PI * freq * t + phase);
			}
		}
	}
}

void ConstrainedDisturbanceGenerator::cleanTrajectory(TrajectoryKinematics& trajectory) {
	// --- Clip jerk ---
	for (int i = 0; i < trajectory.jerk.rows(); ++i) {
		for (int j = 0; j < 3; ++j) {
			double limit = (1.0 - constraints.safety_margin) * constraints.max_jerk(j);
			trajectory.jerk(i, j) = std::clamp(trajectory.jerk(i, j), -limit, limit);
		}
	}

	// --- Integrate jerk to get acceleration ---
	trajectory.acceleration = KinematicsCalculator::integrateSignal(
		trajectory.jerk, trajectory.dt, trajectory.initAcceleration);

	// --- Clip acceleration ---
	for (int i = 0; i < trajectory.acceleration.rows(); ++i) {
		for (int j = 0; j < 3; ++j) {
			double limit = (1.0 - constraints.safety_margin) * constraints.max_acc(j);
			trajectory.acceleration(i, j) = std::clamp(trajectory.acceleration(i, j), -limit, limit);
		}
	}

	// --- Integrate acceleration to get velocity ---
	trajectory.velocity = KinematicsCalculator::integrateSignal(
		trajectory.acceleration, trajectory.dt, trajectory.initVelocity);

	// --- Clip velocity ---
	for (int i = 0; i < trajectory.velocity.rows(); ++i) {
		for (int j = 0; j < 3; ++j) {
			double limit = (1.0 - constraints.safety_margin) * constraints.max_vel(j);
			trajectory.velocity(i, j) = std::clamp(trajectory.velocity(i, j), -limit, limit);
		}
	}

	// --- Integrate velocity to get position ---
	trajectory.position = KinematicsCalculator::integrateSignal(
		trajectory.velocity, trajectory.dt, trajectory.initPosition);

	// --- Clip position within [min + margin, max - margin] ---
	for (int i = 0; i < trajectory.position.rows(); ++i) {
		for (int j = 0; j < 3; ++j) {
			double range = constraints.max_pos(j) - constraints.min_pos(j);
			double margin = constraints.safety_margin * range / 2.0;
			double min_limit = constraints.min_pos(j) + margin;
			double max_limit = constraints.max_pos(j) - margin;
			trajectory.position(i, j) = std::clamp(trajectory.position(i, j), min_limit, max_limit);
		}
	}
	//KinematicsCalculator::computeKinematicDerivatives(trajectory);
}

//TrajectoryKinematics ConstrainedDisturbanceGenerator::cleanTrajectory_v2(const TrajectoryKinematics& trajectory) {
//	TrajectoryKinematics cleanedTrajectory = trajectory;
//	// --- Clip jerk ---
//	for (int i = 0; i < trajectory.jerk.rows(); ++i) {
//		for (int j = 0; j < 3; ++j) {
//			double limit = (1.0 - constraints.safety_margin) * constraints.max_jerk(j);
//			cleanedTrajectory.jerk(i, j) = std::clamp(trajectory.jerk(i, j), -limit, limit);
//		}
//	}
//
//	// --- Integrate jerk to get acceleration ---
//	cleanedTrajectory.acceleration = KinematicsCalculator::integrateSignal(
//		cleanedTrajectory.jerk, cleanedTrajectory.dt, cleanedTrajectory.initAcceleration);
//
//	// --- Clip acceleration ---
//	for (int i = 0; i < cleanedTrajectory.acceleration.rows(); ++i) {
//		for (int j = 0; j < 3; ++j) {
//			double limit = (1.0 - constraints.safety_margin) * constraints.max_acc(j);
//			cleanedTrajectory.acceleration(i, j) = std::clamp(cleanedTrajectory.acceleration(i, j), -limit, limit);
//		}
//	}
//
//	// --- Integrate acceleration to get velocity ---
//	cleanedTrajectory.velocity = KinematicsCalculator::integrateSignal(
//		cleanedTrajectory.acceleration, cleanedTrajectory.dt, cleanedTrajectory.initVelocity);
//
//	// --- Clip velocity ---
//	for (int i = 0; i < cleanedTrajectory.velocity.rows(); ++i) {
//		for (int j = 0; j < 3; ++j) {
//			double limit = (1.0 - constraints.safety_margin) * constraints.max_vel(j);
//			cleanedTrajectory.velocity(i, j) = std::clamp(cleanedTrajectory.velocity(i, j), -limit, limit);
//		}
//	}
//
//	// --- Integrate velocity to get position ---
//	cleanedTrajectory.position = KinematicsCalculator::integrateSignal(
//		cleanedTrajectory.velocity, cleanedTrajectory.dt, cleanedTrajectory.initPosition);
//
//	// --- Clip position within [min + margin, max - margin] ---
//	for (int i = 0; i < cleanedTrajectory.position.rows(); ++i) {
//		for (int j = 0; j < 3; ++j) {
//			double range = constraints.max_pos(j) - constraints.min_pos(j);
//			double margin = constraints.safety_margin * range / 2.0;
//			double min_limit = constraints.min_pos(j) + margin;
//			double max_limit = constraints.max_pos(j) - margin;
//			cleanedTrajectory.position(i, j) = std::clamp(cleanedTrajectory.position(i, j), min_limit, max_limit);
//		}
//	}
//	KinematicsCalculator::computeKinematicDerivatives(cleanedTrajectory);
//	return cleanedTrajectory;
//}

