#pragma once
#include "core/system_constants.hpp"

struct MotionConfig {
    // Physical limits
    double maxJerk = SystemConstants::Machine::MAX_JERK.x();  // Assuming uniform across axes
    double maxAcceleration = SystemConstants::Machine::MAX_ACCELERATION.x();
    double maxVelocity = SystemConstants::Machine::MAX_VELOCITY.x();

    // Control parameters
    double controllerFrequency = SystemConstants::Timing::CONTROLLER_FREQUENCY_HZ;

    // Rapid move multipliers (for G0 commands)
    double rapidAccelMultiplier = 1.5;
    double rapidVelocityMultiplier = 1.2;

    // Tolerance settings
    double positionTolerance = 1e-6;    // mm
    double velocityTolerance = 1e-3;    // mm/s

    // Derived values
    double getTimeStep() const {
        return SystemConstants::Timing::CONTROLLER_TIME_STEP_S;
    }

    // Validation
    bool isValid() const {
        return maxJerk > 0 &&
            maxAcceleration > 0 &&
            maxVelocity > 0 &&
            controllerFrequency > 0 &&
            rapidAccelMultiplier > 0 &&
            rapidVelocityMultiplier > 0;
    }

    // Get rapid motion limits
    MotionConfig getRapidConfig() const {
        MotionConfig rapid = *this;
        rapid.maxAcceleration *= rapidAccelMultiplier;
        rapid.maxVelocity *= rapidVelocityMultiplier;
        return rapid;
    }
};
