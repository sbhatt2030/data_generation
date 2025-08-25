#pragma once
#include <Eigen/Dense>

/**
 * CENTRAL SYSTEM CONSTANTS
 * Single source of truth for all system-wide configuration values
 *
 * IMPORTANT: Only modify values here - do not duplicate elsewhere!
 */

namespace SystemConstants {

    // =============================================================================
    // MOTION CONTROL TIMING
    // =============================================================================

    namespace Timing {
        constexpr double CONTROLLER_FREQUENCY_HZ = 4000.0;     // Real-time controller frequency
        constexpr double MAIN_LOOP_FREQUENCY_HZ = 100;       // Application main loop frequency  
        constexpr double LOGGING_FREQUENCY_HZ = 0.1;           // Status logging frequency

        // Derived values
        constexpr double CONTROLLER_TIME_STEP_S = 1.0 / CONTROLLER_FREQUENCY_HZ;  // 0.00025s
        constexpr int LOGGING_INTERVAL_CYCLES = static_cast<int>(MAIN_LOOP_FREQUENCY_HZ / LOGGING_FREQUENCY_HZ);  // 200 cycles
    }

    // =============================================================================
    // BUFFER CONFIGURATION  
    // =============================================================================

    namespace Buffers {
        constexpr size_t TRAJECTORY_CHUNK_SIZE = 8000;         // Points per trajectory chunk
        constexpr size_t INJECTION_BUFFER_CAPACITY = 16000;    // Maximum injection buffer size
        constexpr size_t INJECTION_REFILL_TRIGGER = 8000;      // Refill when buffer drops below this
        constexpr size_t EXTRACTION_BUFFER_SIZE = 8000;        // Fixed extraction buffer size
        constexpr size_t SMR_SHARED_MEMORY_SIZE = 8000;        // SMR buffer size (must match extraction)

        // Logging intervals
        constexpr size_t INJECTION_LOGGING_INTERVAL = 8000;    // Log every N points injected
        constexpr size_t EXTRACTION_LOGGING_INTERVAL = 1000;   // Log every N points extracted
    }

    // =============================================================================
    // MACHINE CONSTRAINTS (MASTER VALUES)
    // =============================================================================

    namespace Machine {
        // Position limits [mm] - These should already include any safety margins you want
        const Eigen::Vector3d WORKSPACE_MIN(-510, -290, -240);
        const Eigen::Vector3d WORKSPACE_MAX(510, 290, 240);

        // Motion limits [SI units: mm/s, mm/s², mm/s³] - These should already include any safety margins you want
        const Eigen::Vector3d MAX_VELOCITY(533.0, 533.0, 533.0);      // mm/s
        const Eigen::Vector3d MAX_ACCELERATION(2450.0, 2450.0, 2450.0); // mm/s²
        const Eigen::Vector3d MAX_JERK(12250.0, 12250.0, 12250.0);    // mm/s³

        // G-code limits [mm/min for feedrate]
        constexpr double MAX_FEEDRATE_MM_PER_MIN = 32000.0;    // 32000 mm/min maximum

        // Position validation margin for G-code generation
        constexpr double POSITION_VALIDATION_MARGIN_MM = 10.0; // Margin for position validation
    }

    // =============================================================================
    // NOISE GENERATION SAFETY LIMITS
    // =============================================================================

    namespace NoiseLimits {
        constexpr double MAX_FOLLOWING_ERROR_MM = 1.0;         // Hard system limit - DO NOT EXCEED
        constexpr double MAX_NOISE_DEVIATION_MM = 0.9;         // Noise generator safety limit
        constexpr double SAFE_INJECTION_MARGIN_MM = 0.2;       // Safety buffer below following error limit

        // Default noise parameters
        constexpr double DEFAULT_AMPLITUDE_MM = 0.005;         // 5 micrometers default
        constexpr double DEFAULT_FREQUENCY_CUTOFF_HZ = 25.0;   // Default cutoff frequency
        constexpr double DEFAULT_SMOOTHNESS_FACTOR = 0.7;      // Temporal smoothness
        constexpr double DEFAULT_DAMPING_RATIO = 0.8;          // For servo-like behavior

        // Sparse injection limits
        constexpr double MIN_INJECTION_AMPLITUDE_MM = 0.01;    // 10 micrometers minimum
        constexpr double MAX_INJECTION_AMPLITUDE_MM = 0.5;     // 500 micrometers maximum
        constexpr double DEFAULT_INJECTION_PROBABILITY = 0.03; // 3% injection rate
    }

    // =============================================================================
    // G-CODE GENERATION DEFAULTS
    // =============================================================================

    namespace GCodeDefaults {
        constexpr double MAX_TRAJECTORY_TIME_S = 2.0;          // Maximum time per trajectory
        constexpr double INTER_TRAJECTORY_DWELL_S = 0.5;       // Dwell between trajectories
        constexpr double INITIAL_DWELL_S = 5.0;                // Initial program dwell
        constexpr double LINEAR_PROBABILITY = 0.6;             // 60% linear moves in mixed mode

        // Rapid move settings
        constexpr double RAPID_FEEDRATE_MM_PER_MIN = 5000.0;   // Rapid positioning feedrate
        constexpr double MIN_PRACTICAL_FEEDRATE_MM_PER_MIN = 100.0; // Minimum feedrate

        // Arc constraints
        constexpr double MIN_ARC_RADIUS_MM = 5.0;              // Minimum arc radius
        constexpr double MAX_ARC_RADIUS_MM = 50.0;             // Maximum arc radius
        constexpr double MIN_ARC_DEPTH_MM = 0.005;             // Minimum arc depth
        constexpr double MIN_ARC_ANGLE_DEG = 30.0;             // Minimum arc angle
        constexpr double MAX_ARC_ANGLE_DEG = 180.0;            // Maximum arc angle

        // Move distance constraints
        constexpr double MIN_MOVE_DISTANCE_MM = 10.0;          // Minimum linear move distance
        constexpr double MAX_MOVE_DISTANCE_MM = 200.0;         // Maximum linear move distance
    }

    // =============================================================================
    // VFF GENERATION DEFAULTS
    // =============================================================================

    namespace VFFDefaults {
        constexpr double MIN_AMPLITUDE = 0.1;                  // Minimum VFF amplitude
        constexpr double MAX_AMPLITUDE = 10.0;                 // Maximum VFF amplitude
        constexpr double MIN_ALPHA = 0.01;                     // Minimum exponential smoothing alpha
        constexpr double MAX_ALPHA = 0.2;                      // Maximum exponential smoothing alpha
        constexpr double SPARSE_VFF_PROBABILITY = 0.02;        // Sparse VFF injection probability
        constexpr double MIN_SPARSE_VFF_AMPLITUDE = 1.0;       // Minimum sparse VFF amplitude
        constexpr double MAX_SPARSE_VFF_AMPLITUDE = 25.0;      // Maximum sparse VFF amplitude
    }

    // =============================================================================
    // UTILITY FUNCTIONS
    // =============================================================================

    namespace Utils {
        // Unit conversions
        inline double mmPerSecToMmPerMin(double mm_per_s) {
            return mm_per_s * 60.0;
        }

        inline double mmPerMinToMmPerSec(double mm_per_min) {
            return mm_per_min / 60.0;
        }

        // Safety validation
        inline bool isNoiseAmplitudeSafe(double amplitude_mm) {
            return amplitude_mm <= NoiseLimits::MAX_NOISE_DEVIATION_MM;
        }

        /*inline double getSafeInjectionLimit(double predicted_following_error_mm) {
            return std::max(0.0, NoiseLimits::MAX_FOLLOWING_ERROR_MM -
                NoiseLimits::SAFE_INJECTION_MARGIN_MM - predicted_following_error_mm);
        }*/
    }

} // namespace SystemConstants