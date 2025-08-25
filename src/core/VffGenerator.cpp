#define _USE_MATH_DEFINES
#include <cmath>
#include "core/VffGenerator.hpp"
#include <iostream>
#include <random>
#include <iomanip>
#include <algorithm>

VffGenerator::VffGenerator(unsigned int seed) {
    if (seed == 0) {
        std::random_device rd;
        rng_.seed(rd());
    }
    else {
        rng_.seed(seed);
    }

    resetContinuousState();
}

std::array<std::vector<double>, 3> VffGenerator::generateVffChunk(
    int chunkSize,
    VffType vffType,
    const VffParams& params,
    double dt) {

    // Initialize state if first chunk
    if (!continuousState_.initialized) {
        resetContinuousState();
        continuousState_.initialized = true;
    }

    std::cout << "Generating VFF chunk: " << chunkSize << " samples, type " << static_cast<int>(vffType)
        << ", max_amplitude " << params.max_amplitude << std::endl;

    std::array<std::vector<double>, 3> result;

    // Generate VFF based on type
    switch (vffType) {
    case VffType::SMOOTH_GAUSSIAN:
        result = generateSmoothGaussian(chunkSize, params, dt);
        break;
    case VffType::SMOOTH_GAUSSIAN_DC_SHIFT:
        result = generateSmoothGaussianDCShift(chunkSize, params, dt);
        break;
    case VffType::SPARSE_VFF:
        result = generateSparseVff(chunkSize, params, dt);
        break;
    case VffType::NO_VFF:
    default:

        /*result = generateVffTestPattern(chunkSize, params, dt);*/
        // Return zero-filled vectors
        for (int axis = 0; axis < 3; ++axis) {
            result[axis].resize(chunkSize, 0.0);
        }
        break;
    }

    continuousState_.chunkCounter++;
    std::cout << "Generated VFF chunk successfully" << std::endl;
    return result;
}

void VffGenerator::addVffToChunk(
    std::vector<InputDataPoint>& chunk,
    VffType vffType,
    const VffParams& params,
    double dt) {

    if (chunk.empty()) return;

    // Generate VFF signals for the chunk
    std::array<std::vector<double>, 3> vffSignals = generateVffChunk(
        static_cast<int>(chunk.size()), vffType, params, dt);

    // Add VFF data to each point in the chunk
    for (size_t i = 0; i < chunk.size(); ++i) {
        chunk[i].vff_x = vffSignals[0][i];
        chunk[i].vff_y = vffSignals[1][i];
        chunk[i].vff_z = vffSignals[2][i];
    }

    std::cout << "Added VFF data to chunk" << std::endl;
}

void VffGenerator::resetContinuousState() {
    continuousState_.initialized = false;
    continuousState_.chunkCounter = 0;

    // Reset Butterworth filter state
    butterworth_state_.initialized = false;
    for (int axis = 0; axis < 3; ++axis) {
        butterworth_state_.x_history[axis][0] = butterworth_state_.x_history[axis][1] = 0.0;
        butterworth_state_.y_history[axis][0] = butterworth_state_.y_history[axis][1] = 0.0;
    }

    std::cout << "Reset continuous VFF generation state" << std::endl;
}

std::array<std::vector<double>, 3> VffGenerator::generateSmoothGaussian(
    int chunkSize, const VffParams& params, double dt) {

    std::array<std::vector<double>, 3> result;

    // σ = max_amplitude/3 (ignore min_amplitude)
    double sigma = params.max_amplitude / 3.0;

    // Generate for each axis
    for (int axis = 0; axis < 3; ++axis) {
        result[axis].resize(chunkSize);

        for (int i = 0; i < chunkSize; ++i) {
            // Generate white Gaussian noise
            result[axis][i] = generateGaussianNoise(0.0, sigma);
        }
    }

    // Apply 2nd order Butterworth low-pass filter at max_frequency
    for (int i = 0; i < chunkSize; ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            applyButterworthFilterSingleAxis(result[axis][i], axis, params.max_frequency, dt);
        }
    }

    return result;
}

std::array<std::vector<double>, 3> VffGenerator::generateSmoothGaussianDCShift(
    int chunkSize, const VffParams& params, double dt) {

    std::array<std::vector<double>, 3> result;

    // Generate random DC shift for each axis: uniform [min_dc_shift, max_dc_shift]
    std::uniform_real_distribution<double> dc_dist(params.min_dc_shift, params.max_dc_shift);
    std::array<double, 3> dc_shifts;
    for (int axis = 0; axis < 3; ++axis) {
        dc_shifts[axis] = dc_dist(rng_);
    }

    // Generate for each axis
    for (int axis = 0; axis < 3; ++axis) {
        result[axis].resize(chunkSize);

        // σ = (max_amplitude - |DC_shift|) / 3
        double sigma = (params.max_amplitude - std::abs(dc_shifts[axis])) / 3.0;

        // Ensure sigma is positive
        if (sigma <= 0.0) {
            sigma = 0.001; // Minimal noise if DC shift is too large
        }

        for (int i = 0; i < chunkSize; ++i) {
            // Generate Gaussian noise + DC shift
            result[axis][i] = generateGaussianNoise(0.0, sigma) + dc_shifts[axis];
        }
    }

    // Apply 2nd order Butterworth low-pass filter at max_frequency
    for (int i = 0; i < chunkSize; ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            applyButterworthFilterSingleAxis(result[axis][i], axis, params.max_frequency, dt);
        }
    }

    std::cout << "Generated VFF with DC shifts: X=" << dc_shifts[0]
        << ", Y=" << dc_shifts[1] << ", Z=" << dc_shifts[2] << std::endl;

    return result;
}

std::array<std::vector<double>, 3> VffGenerator::generateSparseVff(
    int chunkSize, const VffParams& params, double dt) {

    std::array<std::vector<double>, 3> result;

    // Initialize all to zero
    for (int axis = 0; axis < 3; ++axis) {
        result[axis].resize(chunkSize, 0.0);
    }

    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
    std::uniform_real_distribution<double> amplitude_dist(params.min_dc_shift, params.max_dc_shift);
    std::uniform_real_distribution<double> polarity_dist(-1.0, 1.0);

    int injection_count = 0;
    for (int i = 0; i < chunkSize; ++i) {
        if (prob_dist(rng_) < params.sparse_probability) {
            // Apply sparse injection
            for (int axis = 0; axis < 3; ++axis) {
                double amplitude = amplitude_dist(rng_);
                double polarity = (polarity_dist(rng_) >= 0.0) ? 1.0 : -1.0;
                result[axis][i] = amplitude * polarity;
            }
            injection_count++;
        }
        // else: result[axis][i] remains 0.0
    }

    std::cout << "Generated sparse VFF with " << injection_count << " injections ("
        << std::fixed << std::setprecision(1)
        << (100.0 * injection_count / chunkSize) << "% of samples)" << std::endl;

    return result;
}

void VffGenerator::applyButterworthFilterSingleAxis(double& sample, int axis, double cutoff_freq, double dt) {
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

double VffGenerator::generateGaussianNoise(double mean, double std_dev) const {
    std::normal_distribution<double> dist(mean, std_dev);
    return dist(rng_);
}

// ============================================================================
// Legacy interface methods (simplified for compatibility)
// ============================================================================

std::array<std::vector<double>, 3> VffGenerator::generateAllAxes() {
    // Legacy method - use default parameters
    VffParams defaultParams;
    return generateVffChunk(signalLength_, VffType::SMOOTH_GAUSSIAN, defaultParams, 0.00025);
}

std::array<std::vector<double>, 3> VffGenerator::generateAllAxes(
    const std::array<double, 3>& amplitudes,
    const std::array<double, 3>& alphas) {

    std::array<std::vector<double>, 3> result;

    for (int axis = 0; axis < 3; ++axis) {
        double amplitude = (amplitudes[axis] > 0.0) ? amplitudes[axis] : getRandomAmplitude();
        double alpha = (alphas[axis] > 0.0) ? alphas[axis] : getRandomAlpha();

        result[axis] = generateType1(amplitude, alpha); // Default to TYPE_1
    }

    return result;
}

std::vector<double> VffGenerator::generateType1(double amplitude, double alpha) const {
    // Type 1: Fixed amplitude + smooth noise (10% of amplitude)
    std::vector<double> smoothNoise = generateSmoothedNoise(alpha);

    // Scale smooth noise to 10% of amplitude
    double noiseAmplitude = 0.1 * amplitude;

    std::vector<double> result(signalLength_);
    for (int i = 0; i < signalLength_; ++i) {
        result[i] = amplitude + (smoothNoise[i] * noiseAmplitude);
    }

    return result;
}

std::vector<double> VffGenerator::generateType2(double amplitude, double alpha) const {
    // Type 2: Pure smooth random signal
    std::vector<double> smoothNoise = generateSmoothedNoise(alpha);

    // Scale to desired amplitude
    for (double& value : smoothNoise) {
        value *= amplitude;
    }

    return smoothNoise;
}

std::vector<double> VffGenerator::generateType3(double amplitude) const {
    std::vector<double> result(signalLength_, 0.0);  // Start with zeros

    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
    std::uniform_real_distribution<double> amplitude_dist(minAmplitude_, maxAmplitude_);
    std::uniform_real_distribution<double> polarity_dist(-1.0, 1.0);

    for (int i = 0; i < signalLength_; ++i) {
        if (prob_dist(rng_) < sparseProbability_) {
            double injection_amplitude = amplitude_dist(rng_);
            double polarity = (polarity_dist(rng_) >= 0.0) ? 1.0 : -1.0;
            result[i] = injection_amplitude * polarity;
        }
        // else: result[i] remains 0.0
    }

    return result;
}

double VffGenerator::getRandomAmplitude() const {
    std::uniform_real_distribution<double> dist(minAmplitude_, maxAmplitude_);
    return dist(rng_);
}

double VffGenerator::getRandomAlpha() const {
    std::uniform_real_distribution<double> dist(minAlpha_, maxAlpha_);
    return dist(rng_);
}

std::vector<double> VffGenerator::generateSmoothedNoise(double alpha) const {
    std::vector<double> result(signalLength_);
    std::uniform_real_distribution<double> noiseDist(-1.0, 1.0);

    // Initialize first sample
    result[0] = noiseDist(rng_);

    // Apply exponential smoothing: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    for (int i = 1; i < signalLength_; ++i) {
        double newNoise = noiseDist(rng_);
        result[i] = (1.0 - alpha) * newNoise + alpha * result[i - 1];
    }

    return result;
}

std::array<std::vector<double>, 3> VffGenerator::generateVffTestPattern(
    int chunkSize, const VffParams& params, double dt) {

    std::array<std::vector<double>, 3> result;

    // VFF test pattern configuration (10x larger than deviation pattern)
    const double minValue = 0.00001;  // 10x larger than deviation
    const double maxValue = 0.08000;  // 10x larger than deviation
    const int cycleLength = 16000;   // Different cycle length for VFF

    std::cout << "Generating VFF test pattern: " << chunkSize << " samples" << std::endl;
    std::cout << "  VFF range: " << std::scientific << std::setprecision(4)
        << minValue << " to " << maxValue << std::endl;

    // Calculate starting sample index for this chunk
    int startSampleIndex = continuousState_.chunkCounter * chunkSize;

    // Generate for each axis
    for (int axis = 0; axis < 3; ++axis) {
        result[axis].resize(chunkSize);

        // Different phase shift for each axis
        int phaseShift = axis * (cycleLength / 3); // 33% phase shift between axes

        for (int i = 0; i < chunkSize; ++i) {
            int globalSampleIndex = startSampleIndex + i + phaseShift;
            int cyclePosition = globalSampleIndex % cycleLength;

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

            result[axis][i] = value;
        }
    }

    // Debug output for first chunk
    if (continuousState_.chunkCounter == 0) {
        std::cout << "VFF test pattern first chunk samples:" << std::endl;
        for (int i = 0; i < std::min(10, chunkSize); i += 2) {
            std::cout << "  Sample " << i << ": X=" << std::scientific << std::setprecision(4)
                << result[0][i] << ", Y=" << result[1][i] << ", Z=" << result[2][i] << std::endl;
        }
    }

    return result;
}