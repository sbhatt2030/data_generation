#include "core/trajectory_generation.hpp"
#include "core/kinematics.hpp"
#include "core/SCurveGeneration.hpp"
#define _USE_MATH_DEFINES
#include <math.h>


TrajectoryGenerator::TrajectoryGenerator(double controllerFrequencyHz, double length) {
	dt = 1.0 / controllerFrequencyHz;
	trajectoryLength = length;
}

TrajectoryKinematics TrajectoryGenerator::generateTrajectory(GCodeCommand cmd) {
    TrajectoryKinematics trajectory;
	switch (cmd.motion) {
	case MotionMode::G0:
		break;
	case MotionMode::G1:
		trajectory = generateLinearTrajectory(cmd);
		break;
	case MotionMode::G2:
	case MotionMode::G3:
		trajectory =  generateCircularTrajectory(cmd);
		break;
	default:
		break;
	}
	return trajectory;
}

//Eigen::MatrixXd TrajectoryGenerator::generateLinearTrajectory(GCodeCommand cmd) {
//	Eigen::Vector3d end(
//		cmd.x.value_or(cmd.startPoint(0)),
//		cmd.y.value_or(cmd.startPoint(1)),
//		cmd.z.value_or(cmd.startPoint(2))
//	);
//
//	Eigen::Vector3d delta = end - cmd.startPoint;
//	double distance = delta.norm();
//
//	// Feedrate in mm/min → mm/s
//	double feedrate = cmd.f.value_or(1000.0)/60.0;
//	double totalTime = distance / feedrate;
//
//	int numSteps = static_cast<int>(std::ceil(totalTime / dt));
//	numSteps = std::min(numSteps, static_cast<int>(trajectoryLength)); // don't exceed 8000
//
//	Eigen::MatrixXd trajectory(trajectoryLength, 3);
//
//	// Generate points
//	for (int i = 0; i < numSteps; ++i) {
//		double alpha = static_cast<double>(i) / (numSteps - 1);
//		trajectory.row(i) = cmd.startPoint + alpha * delta;
//	}
//
//	// Pad remaining rows with final position
//	for (int i = numSteps; i < trajectoryLength; ++i) {
//		trajectory.row(i) = end;
//	}
//
//	return trajectory;
//}
//
//
//Eigen::MatrixXd TrajectoryGenerator::generateCircularTrajectory(GCodeCommand cmd) {
//    Eigen::MatrixXd trajectory(trajectoryLength, 3);
//
//    Eigen::Vector3d start = cmd.startPoint;
//    Eigen::Vector3d end(cmd.x.value_or(start[0]),
//        cmd.y.value_or(start[1]),
//        cmd.z.value_or(start[2]));
//
//    int idx1 = 0, idx2 = 1, staticIdx = 2; // Defaults for XY plane
//    switch (cmd.plane) {
//    case PlaneMode::ZX: idx1 = 0; idx2 = 2; staticIdx = 1; break;
//    case PlaneMode::YZ: idx1 = 1; idx2 = 2; staticIdx = 0; break;
//    default: break;
//    }
//
//    // Compute center
//    Eigen::Vector3d center = start;
//    center[idx1] += cmd.i.value_or(0.0);
//    center[idx2] += cmd.j.value_or(0.0);
//
//    Eigen::Vector2d v0(start[idx1] - center[idx1], start[idx2] - center[idx2]);
//    Eigen::Vector2d v1(end[idx1] - center[idx1], end[idx2] - center[idx2]);
//
//    double radius = v0.norm();
//
//    // Compute signed angle using atan2
//    double cross = v0.x() * v1.y() - v0.y() * v1.x(); // scalar cross
//    double dot = v0.dot(v1);
//    double angle = std::atan2(cross, dot); // (-π, π)
//
//    // Adjust for direction
//    if (cmd.motion == MotionMode::G2 && angle > 0)
//        angle -= 2 * M_PI;
//    if (cmd.motion == MotionMode::G3 && angle < 0)
//        angle += 2 * M_PI;
//
//    // Time and step calculations
//    double arcLength = std::abs(angle * radius);
//    double totalTime = arcLength / (cmd.f.value_or(1000.0) / 60.0);
//    int numSteps = static_cast<int>(std::ceil(totalTime / dt));
//    if (numSteps > trajectoryLength) numSteps = trajectoryLength;
//
//    for (int step = 0; step < numSteps; ++step) {
//        double t = static_cast<double>(step) / (numSteps - 1);
//        double theta = t * angle;
//
//        double cos_theta = std::cos(theta);
//        double sin_theta = std::sin(theta);
//
//        Eigen::Vector2d dir(
//            v0.normalized().x() * cos_theta - v0.normalized().y() * sin_theta,
//            v0.normalized().x() * sin_theta + v0.normalized().y() * cos_theta
//        );
//
//        Eigen::Vector3d point = center;
//        point[idx1] += radius * dir.x();
//        point[idx2] += radius * dir.y();
//        point[staticIdx] = start[staticIdx]; // Keep the static axis constant
//
//        trajectory.row(step) = point.transpose();
//    }
//
//    // Pad with last point
//    if (numSteps < trajectoryLength) {
//        Eigen::RowVector3d lastPoint = trajectory.row(numSteps - 1);
//        for (int i = numSteps; i < trajectoryLength; ++i)
//            trajectory.row(i) = lastPoint;
//    }
//
//    return trajectory;
//}

//Eigen::MatrixXd TrajectoryGenerator::generateLinearTrajectory(GCodeCommand cmd) {
//    Eigen::Vector3d end(
//        cmd.x.value_or(cmd.startPoint(0)),
//        cmd.y.value_or(cmd.startPoint(1)),
//        cmd.z.value_or(cmd.startPoint(2))
//    );
//
//    Eigen::Vector3d delta = end - cmd.startPoint;
//    double distance = delta.norm();
//    Eigen::Vector3d direction = delta.normalized();
//
//    double feedrate = cmd.f.value_or(1000.0) / 60.0; // mm/s
//	SCurveProfile profile = generateSCurveProfile(distance, 12250, 2450, std::min(533.0,feedrate), 0.00025);
//
//    Eigen::MatrixXd trajectory(trajectoryLength, 3);
//    int i = 0;
//    for (; i < profile.positions.size() && i < trajectoryLength; ++i) {
//        trajectory.row(i) = cmd.startPoint + direction * profile.positions[i];
//    }
//    for (; i < trajectoryLength; ++i) {
//        trajectory.row(i) = end;
//    }
//
//    return trajectory;
//}

TrajectoryKinematics TrajectoryGenerator::generateLinearTrajectory(GCodeCommand cmd) {
    TrajectoryKinematics result;

    Eigen::Vector3d end(
        cmd.x.value_or(cmd.startPoint(0)),
        cmd.y.value_or(cmd.startPoint(1)),
        cmd.z.value_or(cmd.startPoint(2))
    );

    Eigen::Vector3d delta = end - cmd.startPoint;
    double distance = delta.norm();
    Eigen::Vector3d direction = delta.normalized();

    double feedrate = cmd.f.value_or(1000.0) / 60.0; // mm/s
    SCurveProfile profile = generateSCurveProfile(distance, 12250, 2450, std::min(533.0, feedrate), result.dt);

    int i = 0;
    for (; i < profile.positions.size() && i < result.position.rows(); ++i) {
        double s = profile.positions[i];
        Eigen::Vector3d pos = cmd.startPoint + direction * s;
        Eigen::Vector3d vel = direction * profile.velocities[i];
        Eigen::Vector3d acc = direction * profile.accelerations[i];
        Eigen::Vector3d jer = direction * profile.jerks[i];

        result.position.row(i) = pos.transpose();
        result.velocity.row(i) = vel.transpose();
        result.acceleration.row(i) = acc.transpose();
        result.jerk.row(i) = jer.transpose();
    }

    for (; i < result.position.rows(); ++i) {
        result.position.row(i) = end.transpose();
        result.velocity.row(i).setZero();
        result.acceleration.row(i).setZero();
        result.jerk.row(i).setZero();
    }

    result.initPosition = cmd.startPoint;
    result.initVelocity.setZero();
    result.initAcceleration.setZero();

    return result;
}


//Eigen::MatrixXd TrajectoryGenerator::generateCircularTrajectory(GCodeCommand cmd) {
//    Eigen::MatrixXd trajectory(trajectoryLength, 3);
//
//    Eigen::Vector3d start = cmd.startPoint;
//    Eigen::Vector3d end(
//        cmd.x.value_or(start[0]),
//        cmd.y.value_or(start[1]),
//        cmd.z.value_or(start[2])
//    );
//
//    int idx1 = 0, idx2 = 1, staticIdx = 2;
//    switch (cmd.plane) {
//    case PlaneMode::ZX: idx1 = 0; idx2 = 2; staticIdx = 1; break;
//    case PlaneMode::YZ: idx1 = 1; idx2 = 2; staticIdx = 0; break;
//    default: break;
//    }
//
//    Eigen::Vector3d center = start;
//    center[idx1] += cmd.i.value_or(0.0);
//    center[idx2] += cmd.j.value_or(0.0);
//
//    Eigen::Vector2d v0(start[idx1] - center[idx1], start[idx2] - center[idx2]);
//    Eigen::Vector2d v1(end[idx1] - center[idx1], end[idx2] - center[idx2]);
//
//    double radius = v0.norm();
//    double cross = v0.x() * v1.y() - v0.y() * v1.x();
//    double dot = v0.dot(v1);
//    double angle = std::atan2(cross, dot);
//
//    if (cmd.motion == MotionMode::G2 && angle > 0) angle -= 2 * M_PI;
//    if (cmd.motion == MotionMode::G3 && angle < 0) angle += 2 * M_PI;
//
//    double arcLength = std::abs(angle * radius);
//    double feedrate = cmd.f.value_or(1000.0) / 60.0; // mm/s
//    SCurveProfile profile = generateSCurveProfile(arcLength, 12250, 2450, std::min(533.0, feedrate), 0.00025);
//
//    Eigen::Vector2d v0n = v0.normalized();
//    int i = 0;
//    for (; i < profile.positions.size() && i < trajectoryLength; ++i) {
//        double s = profile.positions[i];
//        double t = s / arcLength;
//        double theta = t * angle;
//
//        double cos_theta = std::cos(theta);
//        double sin_theta = std::sin(theta);
//
//        Eigen::Vector2d dir(
//            v0n.x() * cos_theta - v0n.y() * sin_theta,
//            v0n.x() * sin_theta + v0n.y() * cos_theta
//        );
//
//        Eigen::Vector3d point = center;
//        point[idx1] += radius * dir.x();
//        point[idx2] += radius * dir.y();
//        point[staticIdx] = start[staticIdx];
//
//        trajectory.row(i) = point.transpose();
//    }
//    for (; i < trajectoryLength; ++i) {
//        trajectory.row(i) = trajectory.row(i - 1);
//    }
//
//    return trajectory;
//}
TrajectoryKinematics TrajectoryGenerator::generateCircularTrajectory(GCodeCommand cmd) {
    TrajectoryKinematics result;

    Eigen::Vector3d start = cmd.startPoint;
    Eigen::Vector3d end(
        cmd.x.value_or(start[0]),
        cmd.y.value_or(start[1]),
        cmd.z.value_or(start[2])
    );

    int idx1 = 0, idx2 = 1, staticIdx = 2;
    switch (cmd.plane) {
    case PlaneMode::ZX: idx1 = 0; idx2 = 2; staticIdx = 1; break;
    case PlaneMode::YZ: idx1 = 1; idx2 = 2; staticIdx = 0; break;
    default: break;
    }

    Eigen::Vector3d center = start;
    center[idx1] += cmd.i.value_or(0.0);
    center[idx2] += cmd.j.value_or(0.0);

    Eigen::Vector2d v0(start[idx1] - center[idx1], start[idx2] - center[idx2]);
    Eigen::Vector2d v1(end[idx1] - center[idx1], end[idx2] - center[idx2]);

    double radius = v0.norm();
    double cross = v0.x() * v1.y() - v0.y() * v1.x();
    double dot = v0.dot(v1);
    double angle = std::atan2(cross, dot);

    if (cmd.motion == MotionMode::G2 && angle > 0) angle -= 2 * M_PI;
    if (cmd.motion == MotionMode::G3 && angle < 0) angle += 2 * M_PI;

    double arcLength = std::abs(angle * radius);
    double feedrate = cmd.f.value_or(1000.0) / 60.0;
    SCurveProfile profile = generateSCurveProfile(arcLength, 12250, 2450, std::min(533.0, feedrate), result.dt);

    Eigen::Vector2d v0n = v0.normalized();

    int i = 0;
    for (; i < profile.positions.size() && i < result.position.rows(); ++i) {
        double s = profile.positions[i];
        double t = s / arcLength;
        double theta = t * angle;

        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);

        Eigen::Vector2d dir(
            v0n.x() * cos_theta - v0n.y() * sin_theta,
            v0n.x() * sin_theta + v0n.y() * cos_theta
        );

        Eigen::Vector2d tangent(-dir.y(), dir.x()); // derivative of dir w.r.t theta

        Eigen::Vector3d point = center;
        point[idx1] += radius * dir.x();
        point[idx2] += radius * dir.y();
        point[staticIdx] = start[staticIdx];

        Eigen::Vector3d vel, acc, jer;
        vel.setZero(); acc.setZero(); jer.setZero();

        vel[idx1] = profile.velocities[i] * tangent.x();
        vel[idx2] = profile.velocities[i] * tangent.y();

        acc[idx1] = profile.accelerations[i] * tangent.x();
        acc[idx2] = profile.accelerations[i] * tangent.y();

        jer[idx1] = profile.jerks[i] * tangent.x();
        jer[idx2] = profile.jerks[i] * tangent.y();

        result.position.row(i) = point.transpose();
        result.velocity.row(i) = vel.transpose();
        result.acceleration.row(i) = acc.transpose();
        result.jerk.row(i) = jer.transpose();
    }

    for (; i < result.position.rows(); ++i) {
        result.position.row(i) = end.transpose();
        result.velocity.row(i).setZero();
        result.acceleration.row(i).setZero();
        result.jerk.row(i).setZero();
    }

    result.initPosition = start;
    result.initVelocity.setZero();
    result.initAcceleration.setZero();

    return result;
}

