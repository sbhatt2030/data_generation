#define _USE_MATH_DEFINES
#include <cmath>
#include "core/KinematicNoiseGenerator.hpp"
#include <iostream>
#include <algorithm>
#include <iomanip>

KinematicNoiseGenerator::KinematicNoiseGenerator(const MachineConstraints& constraints,
    const MotionConfig& motionConfig,
    unsigned int seed)
    : constraints_(constraints), motionConfig_(motionConfig), noiseType_(KinematicNoiseType::NO_NOISE) {

    if (seed == 0) {
        std::random_device rd;
        rng_.seed(rd());
    }
    else {
        rng_.seed(seed);
    }

    resetContinuousState();
}

std::vector<InputDataPoint> KinematicNoiseGenerator::generateNoiseChunk(
    int chunkSize,
    const NoiseParams& params,
    double dt,
    int lineNumber) {

    std::vector<InputDataPoint> chunk;
    chunk.reserve(chunkSize);

    // Auto-increment line number if not specified
    if (lineNumber == -1) {
        lineNumber = continuousState_.chunkCounter++;
    }

    // Initialize state if first chunk
    if (!continuousState_.initialized) {
        resetContinuousState();
        continuousState_.initialized = true;
    }

    std::cout << "Generating noise chunk: " << chunkSize << " samples, line " << lineNumber
        << ", max_amplitude " << params.max_amplitude << "mm" << std::endl;

    // Generate each sample in the chunk (each chunk starts fresh)
    for (int i = 0; i < chunkSize; ++i) {
        InputDataPoint sample = generateSingleNoiseSample(i, dt,
            noiseType_, params, lineNumber);
        chunk.push_back(sample);
    }

    // Apply safety constraints
    applySafetyConstraints(chunk, params);
    validateChunkConstraints(chunk, params);

    std::cout << "Generated noise chunk successfully" << std::endl;
    return chunk;
}

void KinematicNoiseGenerator::resetContinuousState() {
    continuousState_.initialized = false;
    continuousState_.chunkCounter = 0;

    // Reset Butterworth filter state
    butterworth_state_.initialized = false;
    for (int axis = 0; axis < 3; ++axis) {
        butterworth_state_.x_history[axis][0] = butterworth_state_.x_history[axis][1] = 0.0;
        butterworth_state_.y_history[axis][0] = butterworth_state_.y_history[axis][1] = 0.0;
    }

    std::cout << "Reset continuous noise generation state" << std::endl;
}

InputDataPoint KinematicNoiseGenerator::generateSingleNoiseSample(
    int sampleIndex,
    double dt,
    KinematicNoiseType noiseType,
    const NoiseParams& params,
    int lineNumber) {

    InputDataPoint sample;
    sample.line_number = lineNumber;
    sample.vff_x = sample.vff_y = sample.vff_z = 0.0; // VFF handled elsewhere

    // Generate noise based on type
    Eigen::Vector3d noiseValue;
    switch (noiseType) {
    case KinematicNoiseType::SMOOTH_GAUSSIAN_BANDPASS:
        noiseValue = generateSmoothGaussianBandpassSample(sampleIndex, dt, params);
        break;
    case KinematicNoiseType::SUM_OF_SINUSOIDS:
        noiseValue = generateSumOfSinusoidsSample(sampleIndex, dt, params);
        break;
    case KinematicNoiseType::SPARSE_INJECTION:
        noiseValue = generateSparseInjectionSample(sampleIndex, dt, params);
        break;
    case KinematicNoiseType::NO_NOISE:
    default:
        //noiseValue = generateTestPatternSample(sampleIndex, dt, params);
        noiseValue.setZero();
        break;
    }

    // Set sample values
    sample.dev_x = noiseValue(0);
    sample.dev_y = noiseValue(1);
    sample.dev_z = noiseValue(2);

    return sample;
}

Eigen::Vector3d KinematicNoiseGenerator::generateSmoothGaussianBandpassSample(
    int sampleIndex, double dt, const NoiseParams& params) {

    // Generate white Gaussian noise: σ = max_amplitude/3
    double sigma = params.max_amplitude / 3.0;
    Eigen::Vector3d noiseValue;

    for (int axis = 0; axis < 3; ++axis) {
        noiseValue(axis) = generateGaussianNoise(0.0, sigma);
    }

    // Apply 2nd order Butterworth low-pass filter at max_frequency
    for (int axis = 0; axis < 3; ++axis) {
        applyButterworthFilterSingleAxis(noiseValue(axis), axis, params.max_frequency, dt);
    }

    return noiseValue;
}

Eigen::Vector3d KinematicNoiseGenerator::generateSumOfSinusoidsSample(
    int sampleIndex, double dt, const NoiseParams& params) {

    Eigen::Vector3d noiseValue;
    double t = sampleIndex * dt;

    for (int axis = 0; axis < 3; ++axis) {
        // Pick random number of sines for this axis/sample
        std::uniform_int_distribution<int> sine_count_dist(params.min_num_sines, params.max_num_sines);
        int num_sines = sine_count_dist(rng_);

        double sum = 0.0;
        for (int sine_idx = 0; sine_idx < num_sines; ++sine_idx) {
            // Random amplitude [min_amplitude, max_amplitude]
            std::uniform_real_distribution<double> amp_dist(params.min_amplitude, params.max_amplitude);
            double amplitude = amp_dist(rng_);

            // Random frequency [min_frequency, max_frequency]  
            std::uniform_real_distribution<double> freq_dist(params.min_frequency, params.max_frequency);
            double frequency = freq_dist(rng_);

            // Random phase [0, 2π]
            std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);
            double phase = phase_dist(rng_);

            sum += amplitude * std::sin(2.0 * M_PI * frequency * t + phase);
        }

        noiseValue(axis) = sum / std::sqrt(static_cast<double>(num_sines)); // Normalize
    }

    return noiseValue;
}

Eigen::Vector3d KinematicNoiseGenerator::generateSparseInjectionSample(
    int sampleIndex, double dt, const NoiseParams& params) {

    Eigen::Vector3d noiseValue;
    noiseValue.setZero();

    // Check for injection with specified probability
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);

    if (prob_dist(rng_) < params.sparse_probability) {
        // Apply injection with random amplitude [min_amplitude, max_amplitude]
        std::uniform_real_distribution<double> amplitude_dist(params.min_amplitude, params.max_amplitude);
        std::uniform_real_distribution<double> polarity_dist(-1.0, 1.0);

        for (int axis = 0; axis < 3; ++axis) {
            double amplitude = amplitude_dist(rng_);
            double polarity = (polarity_dist(rng_) >= 0.0) ? 1.0 : -1.0;
            noiseValue(axis) = amplitude * polarity;
        }
    }

    return noiseValue;
}

void KinematicNoiseGenerator::applyButterworthFilterSingleAxis(double& sample, int axis, double cutoff_freq, double dt) {
    // 2nd order Butterworth low-pass filter
    const double omega_c = 2.0 * M_PI * cutoff_freq;
    const double k = std::tan(omega_c * dt / 2.0);
    const double norm = 1.0 / (1.0 + M_SQRT2 * k + k * k);

    const double b0 = k * k * norm;
    const double b1 = 2.0 * b0;
    const double b2 = b0;
    const double a1 = 2.0 * (k * k - 1.0) * norm;
    const double a2 = (1.0 - M_SQRT2 * k + k * k) * norm;

    double x_current = sample;

    // Apply filter: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    double y_current = b0 * x_current +
        b1 * butterworth_state_.x_history[axis][0] +
        b2 * butterworth_state_.x_history[axis][1] -
        a1 * butterworth_state_.y_history[axis][0] -
        a2 * butterworth_state_.y_history[axis][1];

    // Update history for this axis only
    butterworth_state_.x_history[axis][1] = butterworth_state_.x_history[axis][0];
    butterworth_state_.x_history[axis][0] = x_current;
    butterworth_state_.y_history[axis][1] = butterworth_state_.y_history[axis][0];
    butterworth_state_.y_history[axis][0] = y_current;

    sample = y_current;
    butterworth_state_.initialized = true;
}

double KinematicNoiseGenerator::generateGaussianNoise(double mean, double std_dev) const {
    std::normal_distribution<double> dist(mean, std_dev);
    return dist(rng_);
}

void KinematicNoiseGenerator::applySafetyConstraints(std::vector<InputDataPoint>& chunk, const NoiseParams& params) {
    for (auto& sample : chunk) {
        // Clamp each axis to max_deviation_magnitude
        if (std::abs(sample.dev_x) > params.max_deviation_magnitude) {
            sample.dev_x = std::copysign(params.max_deviation_magnitude, sample.dev_x);
        }
        if (std::abs(sample.dev_y) > params.max_deviation_magnitude) {
            sample.dev_y = std::copysign(params.max_deviation_magnitude, sample.dev_y);
        }
        if (std::abs(sample.dev_z) > params.max_deviation_magnitude) {
            sample.dev_z = std::copysign(params.max_deviation_magnitude, sample.dev_z);
        }
    }
}

void KinematicNoiseGenerator::validateChunkConstraints(std::vector<InputDataPoint>& chunk, const NoiseParams& params) {
    if (chunk.empty()) return;

    double maxDeviation = 0.0;
    double rmsDeviation = 0.0;
    size_t violationCount = 0;

    for (const auto& sample : chunk) {
        double deviation = std::sqrt(sample.dev_x * sample.dev_x +
            sample.dev_y * sample.dev_y +
            sample.dev_z * sample.dev_z);
        maxDeviation = std::max(maxDeviation, deviation);
        rmsDeviation += deviation * deviation;

        if (deviation > params.max_deviation_magnitude) {
            violationCount++;
        }
    }

    rmsDeviation = std::sqrt(rmsDeviation / chunk.size());

    if (violationCount > 0) {
        std::cout << "WARNING: " << violationCount << " constraint violations in chunk" << std::endl;
    }

    std::cout << "Chunk validation: max=" << maxDeviation * 1000 << "μm, rms="
        << rmsDeviation * 1000 << "μm" << std::endl;
}

// ============================================================================
// Legacy interface methods (simplified for compatibility)
// ============================================================================

TrajectoryKinematics KinematicNoiseGenerator::addKinematicNoise(
    const TrajectoryKinematics& original,
    KinematicNoiseType type,
    const NoiseParams& params) const {

    TrajectoryKinematics result;

    switch (type) {
    case KinematicNoiseType::SMOOTH_GAUSSIAN_BANDPASS:
        result = addGaussianFilteredNoise(original, params);
        break;
    case KinematicNoiseType::SUM_OF_SINUSOIDS:
        result = addSmoothPerturbation(original, params);
        break;
    case KinematicNoiseType::SPARSE_INJECTION:
        result = addSparseInjection(original, params);
        break;
    case KinematicNoiseType::NO_NOISE:
    default:
        result = original;
        break;
    }

    return result;
}

TrajectoryKinematics KinematicNoiseGenerator::addGaussianFilteredNoise(
    const TrajectoryKinematics& original,
    const NoiseParams& params) const {

    TrajectoryKinematics noisy = original;
    const int num_points = original.getTimeSteps();
    const double dt = original.dt;

    // Generate filtered Gaussian noise
    double sigma = params.max_amplitude / 3.0;

    // Reset filter state for trajectory
    ButterworthState local_filter_state;

    for (int i = 0; i < num_points; ++i) {
        Eigen::Vector3d noise_sample;
        for (int axis = 0; axis < 3; ++axis) {
            noise_sample(axis) = generateGaussianNoise(0.0, sigma);
        }

        // Apply Butterworth filter (simplified - would need proper state management)
        // For now, just add the noise directly
        noisy.position.row(i) += noise_sample.transpose();
    }

    // Recompute kinematics and enforce constraints
    recomputeKinematics(noisy);
    enforceKinematicConstraints(noisy);

    return noisy;
}

TrajectoryKinematics KinematicNoiseGenerator::addSmoothPerturbation(
    const TrajectoryKinematics& original,
    const NoiseParams& params) const {

    TrajectoryKinematics noisy = original;
    const int num_points = original.getTimeSteps();
    const double dt = original.dt;

    // Generate sum of sinusoids
    for (int axis = 0; axis < 3; ++axis) {
        std::uniform_int_distribution<int> sine_count_dist(params.min_num_sines, params.max_num_sines);
        int num_sines = sine_count_dist(rng_);

        for (int comp = 0; comp < num_sines; ++comp) {
            std::uniform_real_distribution<double> amp_dist(params.min_amplitude, params.max_amplitude);
            std::uniform_real_distribution<double> freq_dist(params.min_frequency, params.max_frequency);
            std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);

            double amplitude = amp_dist(rng_) / std::sqrt(static_cast<double>(num_sines));
            double frequency = freq_dist(rng_);
            double phase = phase_dist(rng_);

            for (int i = 0; i < num_points; ++i) {
                double t = i * dt;
                double perturbation = amplitude * std::sin(2.0 * M_PI * frequency * t + phase);
                noisy.position(i, axis) += perturbation;
            }
        }
    }

    recomputeKinematics(noisy);
    enforceKinematicConstraints(noisy);

    return noisy;
}

TrajectoryKinematics KinematicNoiseGenerator::addSparseInjection(
    const TrajectoryKinematics& original,
    const NoiseParams& params) const {

    TrajectoryKinematics noisy = original;
    const int num_points = original.getTimeSteps();

    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
    std::uniform_real_distribution<double> amplitude_dist(params.min_amplitude, params.max_amplitude);
    std::uniform_real_distribution<double> polarity_dist(-1.0, 1.0);

    for (int i = 0; i < num_points; ++i) {
        if (prob_dist(rng_) < params.sparse_probability) {
            for (int axis = 0; axis < 3; ++axis) {
                double amplitude = amplitude_dist(rng_);
                double polarity = (polarity_dist(rng_) >= 0.0) ? 1.0 : -1.0;
                noisy.position(i, axis) += amplitude * polarity;
            }
        }
    }

    recomputeKinematics(noisy);
    enforceKinematicConstraints(noisy);

    return noisy;
}

// ============================================================================
// Legacy utility methods (minimal implementation)
// ============================================================================

void KinematicNoiseGenerator::recomputeKinematics(TrajectoryKinematics& trajectory) const {
    const double dt = trajectory.dt;
    const int num_points = trajectory.getTimeSteps();

    // Compute velocity using central difference
    for (int i = 1; i < num_points - 1; ++i) {
        trajectory.velocity.row(i) = (trajectory.position.row(i + 1) - trajectory.position.row(i - 1)) / (2.0 * dt);
    }
    trajectory.velocity.row(0) = (trajectory.position.row(1) - trajectory.position.row(0)) / dt;
    trajectory.velocity.row(num_points - 1) = (trajectory.position.row(num_points - 1) - trajectory.position.row(num_points - 2)) / dt;

    // Compute acceleration
    for (int i = 1; i < num_points - 1; ++i) {
        trajectory.acceleration.row(i) = (trajectory.velocity.row(i + 1) - trajectory.velocity.row(i - 1)) / (2.0 * dt);
    }
    trajectory.acceleration.row(0) = (trajectory.velocity.row(1) - trajectory.velocity.row(0)) / dt;
    trajectory.acceleration.row(num_points - 1) = (trajectory.velocity.row(num_points - 1) - trajectory.velocity.row(num_points - 2)) / dt;

    // Compute jerk
    for (int i = 1; i < num_points - 1; ++i) {
        trajectory.jerk.row(i) = (trajectory.acceleration.row(i + 1) - trajectory.acceleration.row(i - 1)) / (2.0 * dt);
    }
    trajectory.jerk.row(0) = (trajectory.acceleration.row(1) - trajectory.acceleration.row(0)) / dt;
    trajectory.jerk.row(num_points - 1) = (trajectory.acceleration.row(num_points - 1) - trajectory.acceleration.row(num_points - 2)) / dt;
}

void KinematicNoiseGenerator::enforceKinematicConstraints(TrajectoryKinematics& trajectory) const {
    applyVelocityConstraints(trajectory.velocity);
    applyAccelerationConstraints(trajectory.acceleration);
    applyJerkConstraints(trajectory.jerk);
}

void KinematicNoiseGenerator::applyVelocityConstraints(Eigen::MatrixXd& velocity) const {
    const Eigen::Vector3d max_vel = constraints_.max_velocity;
    for (int i = 0; i < velocity.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            velocity(i, j) = std::max(-max_vel(j), std::min(max_vel(j), velocity(i, j)));
        }
    }
}

void KinematicNoiseGenerator::applyAccelerationConstraints(Eigen::MatrixXd& acceleration) const {
    const Eigen::Vector3d max_acc = constraints_.max_acceleration;
    for (int i = 0; i < acceleration.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            acceleration(i, j) = std::max(-max_acc(j), std::min(max_acc(j), acceleration(i, j)));
        }
    }
}

void KinematicNoiseGenerator::applyJerkConstraints(Eigen::MatrixXd& jerk) const {
    const Eigen::Vector3d max_jerk = constraints_.max_jerk;
    for (int i = 0; i < jerk.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            jerk(i, j) = std::max(-max_jerk(j), std::min(max_jerk(j), jerk(i, j)));
        }
    }
}

Eigen::Vector3d KinematicNoiseGenerator::generateTestPatternSample(
    int sampleIndex, double dt, const NoiseParams& params) {

    // Test pattern configuration
    const double minValue = 0.00001;
    const double maxValue = 0.08000;
    const int cycleLength = 16000; // 2000 samples for complete up-down cycle

    // Calculate position in cycle (0 to cycleLength-1)
    int cyclePosition = sampleIndex % cycleLength;

    double value;
    if (cyclePosition < cycleLength / 2) {
        // First half: ramp up from min to max
        double progress = static_cast<double>(cyclePosition) / (cycleLength / 2);
        value = minValue + (maxValue - minValue) * progress;
    }
    else {
        // Second half: ramp down from max to min
        double progress = static_cast<double>(cyclePosition - cycleLength / 2) / (cycleLength / 2);
        value = maxValue - (maxValue - minValue) * progress;
    }

    // Apply to all axes with slight phase shifts for variety
    Eigen::Vector3d result;
    result(0) = value; // X axis - base pattern

    // Y axis - 25% phase shift
    int yPhase = (sampleIndex + cycleLength / 4) % cycleLength;
    if (yPhase < cycleLength / 2) {
        double progress = static_cast<double>(yPhase) / (cycleLength / 2);
        result(1) = minValue + (maxValue - minValue) * progress;
    }
    else {
        double progress = static_cast<double>(yPhase - cycleLength / 2) / (cycleLength / 2);
        result(1) = maxValue - (maxValue - minValue) * progress;
    }

    // Z axis - 50% phase shift
    int zPhase = (sampleIndex + cycleLength / 2) % cycleLength;
    if (zPhase < cycleLength / 2) {
        double progress = static_cast<double>(zPhase) / (cycleLength / 2);
        result(2) = minValue + (maxValue - minValue) * progress;
    }
    else {
        double progress = static_cast<double>(zPhase - cycleLength / 2) / (cycleLength / 2);
        result(2) = maxValue - (maxValue - minValue) * progress;
    }

    // Debug output every 1000 samples
    if (sampleIndex % 1000 == 0) {
        std::cout << "Test pattern sample " << sampleIndex
            << ": X=" << std::scientific << std::setprecision(4) << result(0)
            << ", Y=" << result(1) << ", Z=" << result(2) << std::endl;
    }

    return result;
}