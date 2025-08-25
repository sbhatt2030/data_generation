#pragma once
#include <Eigen/Dense>
#include "core/system_constants.hpp"

/**
 * Core trajectory data structure containing kinematic information
 */
struct TrajectoryKinematics {
    static constexpr int FIXED_LENGTH = 8000; // Add this line

    Eigen::MatrixXd position = Eigen::MatrixXd::Zero(FIXED_LENGTH, 3);      // Pre-initialize
    Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(FIXED_LENGTH, 3);      // Pre-initialize
    Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(FIXED_LENGTH, 3);  // Pre-initialize
    Eigen::MatrixXd jerk = Eigen::MatrixXd::Zero(FIXED_LENGTH, 3);         // Pre-initialize

    // Initial conditions
    Eigen::Vector3d initPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d initVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d initAcceleration = Eigen::Vector3d::Zero();

    // Time step
    double dt = 0.00025;

    // Optional deviation tracking
    Eigen::MatrixXd position_deviation = Eigen::MatrixXd::Zero(FIXED_LENGTH, 3);

    // Utility functions
    int getTimeSteps() const { return FIXED_LENGTH; }
    bool isValid() const {
        // Check for NaN or infinite values
        if (!position.allFinite() || !velocity.allFinite() ||
            !acceleration.allFinite() || !jerk.allFinite()) {
            return false;
        }

        // Check time step is positive
        if (dt <= 0) {
            return false;
        }

        // Check matrix dimensions
        return position.rows() == FIXED_LENGTH && position.cols() == 3 &&
            velocity.rows() == FIXED_LENGTH && velocity.cols() == 3 &&
            acceleration.rows() == FIXED_LENGTH && acceleration.cols() == 3 &&
            jerk.rows() == FIXED_LENGTH && jerk.cols() == 3;
    }
};

/**
 * Machine constraints for CNC operations
 */
struct MachineConstraints {
    // Position limits [mm]
    Eigen::Vector3d min_position = SystemConstants::Machine::WORKSPACE_MIN;
    Eigen::Vector3d max_position = SystemConstants::Machine::WORKSPACE_MAX;

    Eigen::Vector3d max_velocity = SystemConstants::Machine::MAX_VELOCITY;
    Eigen::Vector3d max_acceleration = SystemConstants::Machine::MAX_ACCELERATION;
    Eigen::Vector3d max_jerk = SystemConstants::Machine::MAX_JERK;

    // Safety margin factor (0.0 to 1.0)
    double safety_margin = 0.1;
	
    double max_feedrate = SystemConstants::Machine::MAX_FEEDRATE_MM_PER_MIN;

    // Utility functions
    Eigen::Vector3d getEffectiveMaxVelocity() const {
        return (1.0 - safety_margin) * max_velocity;
    }

    Eigen::Vector3d getEffectiveMaxAcceleration() const {
        return (1.0 - safety_margin) * max_acceleration;
    }

    Eigen::Vector3d getEffectiveMaxJerk() const {
        return (1.0 - safety_margin) * max_jerk;
    }
};