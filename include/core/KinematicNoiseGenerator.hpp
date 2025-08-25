#pragma once
#include <random>
#include <vector>
#include <Eigen/Dense>
#include "core/trajectory_types.hpp"
#include "core/motion_config.hpp"
#include "core/input_data_point.hpp"

enum class KinematicNoiseType {
    SMOOTH_GAUSSIAN_BANDPASS = 0,
    SUM_OF_SINUSOIDS = 1,
    SPARSE_INJECTION = 2,
    NO_NOISE = 3
};

class KinematicNoiseGenerator {
public:
    /**
     * Clean noise parameters - maps directly to CSV columns
     */
    struct NoiseParams {
        double min_amplitude = 0.001;        // CSV: noise_min_amplitude
        double max_amplitude = 0.01;         // CSV: noise_max_amplitude  
        double min_frequency = 0.5;          // CSV: noise_min_freq
        double max_frequency = 50.0;         // CSV: noise_max_freq
        int min_num_sines = 3;               // CSV: noise_min_sines
        int max_num_sines = 8;               // CSV: noise_max_sines
        double sparse_probability = 0.03;    // CSV: noise_sparse_prob

        // Safety constraint (system constant, not from CSV)
        double max_deviation_magnitude = 0.9; // mm
    };

    /**
     * Constructor
     */
    explicit KinematicNoiseGenerator(const MachineConstraints& constraints,
        const MotionConfig& motionConfig,
        unsigned int seed = 0);

    /**
     * Generate a chunk of noise samples (8000 points)
     * Each chunk starts fresh - no continuity between chunks
     */
    std::vector<InputDataPoint> generateNoiseChunk(
        int chunkSize,
        const NoiseParams& params,
        double dt,
        int lineNumber = -1);

    /**
     * Reset continuous generation state (for new experiments)
     */
    void resetContinuousState();

    /**
     * Legacy interface for trajectory-based noise (keep for compatibility)
     */
    TrajectoryKinematics addKinematicNoise(
        const TrajectoryKinematics& original,
        KinematicNoiseType type,
        const NoiseParams& params) const;

    void setNoiseType(KinematicNoiseType type) { noiseType_ = type; }

private:
    // Core configuration
    MachineConstraints constraints_;
    MotionConfig motionConfig_;
    mutable std::mt19937 rng_;
    KinematicNoiseType noiseType_;

    // Continuous generation state (minimal - chunks are independent)
    struct ContinuousState {
        bool initialized = false;
        int chunkCounter = 0;
    } continuousState_;

    // Butterworth filter state for Gaussian noise
    mutable struct ButterworthState {
        bool initialized = false;
        std::array<std::array<double, 2>, 3> x_history; // [axis][sample] - input history
        std::array<std::array<double, 2>, 3> y_history; // [axis][sample] - output history

        ButterworthState() {
            for (int axis = 0; axis < 3; ++axis) {
                x_history[axis][0] = x_history[axis][1] = 0.0;
                y_history[axis][0] = y_history[axis][1] = 0.0;
            }
        }
    } butterworth_state_;

    // Core generation methods for single sample
    InputDataPoint generateSingleNoiseSample(
        int sampleIndex,
        double dt,
        KinematicNoiseType noiseType,
        const NoiseParams& params,
        int lineNumber);

    // Type-specific generation methods
    Eigen::Vector3d generateSmoothGaussianBandpassSample(
        int sampleIndex, double dt, const NoiseParams& params);

    Eigen::Vector3d generateSumOfSinusoidsSample(
        int sampleIndex, double dt, const NoiseParams& params);

    Eigen::Vector3d generateSparseInjectionSample(
        int sampleIndex, double dt, const NoiseParams& params);

    // Filtering methods
    void applyButterworthFilterSingleAxis(double& sample, int axis, double cutoff_freq, double dt);

    // Utility methods
    double generateGaussianNoise(double mean, double std_dev) const;
    void validateChunkConstraints(std::vector<InputDataPoint>& chunk, const NoiseParams& params);
    void applySafetyConstraints(std::vector<InputDataPoint>& chunk, const NoiseParams& params);

    // Legacy methods (simplified - keep for compatibility)
    TrajectoryKinematics addGaussianFilteredNoise(const TrajectoryKinematics& original, const NoiseParams& params) const;
    TrajectoryKinematics addSmoothPerturbation(const TrajectoryKinematics& original, const NoiseParams& params) const;
    TrajectoryKinematics addSparseInjection(const TrajectoryKinematics& original, const NoiseParams& params) const;

    // Legacy utility methods
    void recomputeKinematics(TrajectoryKinematics& trajectory) const;
    void enforceKinematicConstraints(TrajectoryKinematics& trajectory) const;
    void applyVelocityConstraints(Eigen::MatrixXd& velocity) const;
    void applyAccelerationConstraints(Eigen::MatrixXd& acceleration) const;
    void applyJerkConstraints(Eigen::MatrixXd& jerk) const;
    Eigen::Vector3d generateTestPatternSample(int sampleIndex, double dt, const NoiseParams& params);
};