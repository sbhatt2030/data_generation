#ifndef TGEN_HPP  
#define TGEN_HPP
#include "core/gcode_parser.hpp"
#include <Eigen/Dense>


struct TrajectoryKinematics {
	Eigen::MatrixXd deviation = Eigen::MatrixXd::Zero(8000, 3);
	Eigen::MatrixXd position = Eigen::MatrixXd::Zero(8000, 3);
	Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(8000, 3);
	Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(8000, 3);
	Eigen::MatrixXd jerk = Eigen::MatrixXd::Zero(8000, 3);
	Eigen::Vector3d initPosition = Eigen::Vector3d::Zero();
	Eigen::Vector3d initVelocity = Eigen::Vector3d::Zero();
	Eigen::Vector3d initAcceleration = Eigen::Vector3d::Zero();
	double dt = 0.00025;
};
class TrajectoryGenerator
{
public:
	TrajectoryGenerator(double controllerFrequencyHz = 4000, double length = 8000);
	TrajectoryKinematics generateTrajectory(GCodeCommand cmd);
private:
	double dt; // time step in seconds 1/ controllerFrequencyHz
	int trajectoryLength; // number of time steps in the trajectory
	//Eigen::MatrixXd generateCircularTrajectory(GCodeCommand cmd); // trajectory matrix (size: length x 3)
	//Eigen::MatrixXd generateLinearTrajectory(GCodeCommand cmd); // trajectory matrix (size: length x 3)
	TrajectoryKinematics generateLinearTrajectory(GCodeCommand cmd); // trajectory matrix (size: length x 3)
	TrajectoryKinematics generateCircularTrajectory(GCodeCommand cmd); // trajectory matrix (size: length x 3)
};


#endif // TGEN_HPP
