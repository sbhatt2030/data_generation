#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <algorithm>
#include "core/VffGenerator.hpp"

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

// ============================================================================
// Public interface
// ============================================================================

std::array<std::vector<double>, 3> VffGenerator::generateVffChunk(
    int chunkSize,
    VffType vffType,
    const VffParams& params,
    double dt)
{
    if (!continuousState_.initialized) {
        resetContinuousState();
        continuousState_.initialized = true;
    }

    std::cout << "Generating VFF chunk: " << chunkSize
        << " samples, type " << static_cast<int>(vffType)
        << ", max_amplitude " << params.max_amplitude << std::endl;

    std::array<std::vector<double>, 3> result;

    switch (vffType) {
    case VffType::SQUARE_WAVE:
        result = generateSquareWave(chunkSize, params);
        break;
    case VffType::SMOOTH_RAMP:
        result = generateSmoothRamp(chunkSize, params);
        break;
    case VffType::SUM_OF_SINUSOIDS:
        result = generateSumOfSinusoids(chunkSize, params, dt);
        break;
    case VffType::NO_VFF:
    case VffType::EXISTING_SEQUENCE:
    default:
        for (int axis = 0; axis < 3; ++axis)
            result[axis].assign(chunkSize, 0.0);
        break;
    }

    continuousState_.chunkCounter++;
    return result;
}

void VffGenerator::addVffToChunk(
    std::vector<InputDataPoint>& chunk,
    VffType vffType,
    const VffParams& params,
    double dt)
{
    if (chunk.empty()) return;

    auto signals = generateVffChunk(static_cast<int>(chunk.size()), vffType, params, dt);

    for (size_t i = 0; i < chunk.size(); ++i) {
        chunk[i].vff_x = signals[0][i];
        chunk[i].vff_y = signals[1][i];
        chunk[i].vff_z = signals[2][i];
    }
}

void VffGenerator::resetContinuousState() {
    continuousState_.initialized = false;
    continuousState_.chunkCounter = 0;
    continuousState_.current_amplitude = { 0.0, 0.0, 0.0 };
    continuousState_.next_amplitude = { 0.0, 0.0, 0.0 };
    continuousState_.control_point = { 0.0, 0.0, 0.0 };
    continuousState_.dwell_remaining = { 0,   0,   0 };
    continuousState_.segment_length = { 0,   0,   0 };
    for (int axis = 0; axis < 3; ++axis) {
        continuousState_.phase[axis] = ContinuousState::Phase::FLAT;
        continuousState_.currentSines[axis].clear();
    }
}

// ============================================================================
// Type 1: Square wave — jump to random amplitude, hold for random dwell
// ============================================================================

std::array<std::vector<double>, 3> VffGenerator::generateSquareWave(
    int chunkSize, const VffParams& params)
{
    std::array<std::vector<double>, 3> result;
    for (int axis = 0; axis < 3; ++axis)
        result[axis].resize(chunkSize);

    for (int axis = 0; axis < 3; ++axis) {
        if (continuousState_.dwell_remaining[axis] <= 0) {
            continuousState_.current_amplitude[axis] = drawAmplitude(params);
            continuousState_.dwell_remaining[axis] = drawDwell(params);
        }

        for (int i = 0; i < chunkSize; ++i) {
            result[axis][i] = continuousState_.current_amplitude[axis];

            continuousState_.dwell_remaining[axis]--;
            if (continuousState_.dwell_remaining[axis] <= 0) {
                continuousState_.current_amplitude[axis] = drawAmplitude(params);
                continuousState_.dwell_remaining[axis] = drawDwell(params);
            }
        }
    }

    return result;
}

// ============================================================================
// Type 2: Smooth ramp — alternates between two phases:
//   RAMP : quadratic Bezier from current_amplitude -> next_amplitude
//   FLAT : constant at current_amplitude
// Both durations drawn from exponential(mean=200, floor=10).
// ============================================================================

std::array<std::vector<double>, 3> VffGenerator::generateSmoothRamp(
    int chunkSize, const VffParams& params)
{
    std::array<std::vector<double>, 3> result;
    for (int axis = 0; axis < 3; ++axis)
        result[axis].resize(chunkSize);

    std::uniform_real_distribution<double> straight_dist(0.0, 1.0);

    for (int axis = 0; axis < 3; ++axis) {

        // Bootstrap
        if (continuousState_.dwell_remaining[axis] <= 0) {
            continuousState_.current_amplitude[axis] = drawAmplitude(params);
            continuousState_.phase[axis] = ContinuousState::Phase::FLAT;
            int len = drawDwell(params);
            continuousState_.dwell_remaining[axis] = len;
            continuousState_.segment_length[axis] = len;
        }

        int i = 0;
        while (i < chunkSize) {
            int toWrite = std::min(continuousState_.dwell_remaining[axis], chunkSize - i);
            int segLen = continuousState_.segment_length[axis];
            int segStart = segLen - continuousState_.dwell_remaining[axis];
            auto& ph = continuousState_.phase[axis];

            for (int s = 0; s < toWrite; ++s) {
                if (ph == ContinuousState::Phase::FLAT) {
                    result[axis][i + s] = continuousState_.current_amplitude[axis];
                }
                else {
                    double t = (segLen > 1)
                        ? static_cast<double>(segStart + s) / static_cast<double>(segLen - 1)
                        : 1.0;
                    t = std::clamp(t, 0.0, 1.0);
                    double u = 1.0 - t;
                    double A = continuousState_.current_amplitude[axis];
                    double B = continuousState_.next_amplitude[axis];
                    double P1 = continuousState_.control_point[axis];
                    result[axis][i + s] = u * u * A + 2.0 * u * t * P1 + t * t * B;
                }
            }

            i += toWrite;
            continuousState_.dwell_remaining[axis] -= toWrite;

            if (continuousState_.dwell_remaining[axis] <= 0) {
                if (ph == ContinuousState::Phase::FLAT) {
                    // Start a ramp toward a new target
                    continuousState_.next_amplitude[axis] = drawAmplitude(params);
                    continuousState_.control_point[axis] = drawControlPoint(
                        continuousState_.current_amplitude[axis],
                        continuousState_.next_amplitude[axis],
                        params, straight_dist(rng_) < 0.2);
                    int len = drawDwell(params);
                    continuousState_.dwell_remaining[axis] = len;
                    continuousState_.segment_length[axis] = len;
                    ph = ContinuousState::Phase::RAMP;
                }
                else {
                    // Ramp done — arrive at next_amplitude, start flat
                    continuousState_.current_amplitude[axis] = continuousState_.next_amplitude[axis];
                    int len = drawDwell(params);
                    continuousState_.dwell_remaining[axis] = len;
                    continuousState_.segment_length[axis] = len;
                    ph = ContinuousState::Phase::FLAT;
                }
            }
        }
    }

    return result;
}

// ============================================================================
// Type 3: Sum of sinusoids — new components drawn each chunk
// ============================================================================

std::array<std::vector<double>, 3> VffGenerator::generateSumOfSinusoids(
    int chunkSize, const VffParams& params, double dt)
{
    std::array<std::vector<double>, 3> result;

    std::uniform_real_distribution<double> amp_dist(0.0, params.max_amplitude);
    std::uniform_real_distribution<double> freq_dist(params.min_frequency, params.max_frequency);
    std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);
    std::uniform_int_distribution<int>     sine_count_dist(params.min_num_sines, params.max_num_sines);

    for (int axis = 0; axis < 3; ++axis) {
        continuousState_.currentSines[axis].clear();
        int num_sines = sine_count_dist(rng_);
        for (int i = 0; i < num_sines; ++i) {
            continuousState_.currentSines[axis].push_back({
                amp_dist(rng_),
                freq_dist(rng_),
                phase_dist(rng_)
                });
        }
    }

    for (int axis = 0; axis < 3; ++axis) {
        result[axis].resize(chunkSize);
        int num_sines = static_cast<int>(continuousState_.currentSines[axis].size());

        // Evaluate raw sum
        for (int i = 0; i < chunkSize; ++i) {
            double t = i * dt;
            double sum = 0.0;
            for (const auto& s : continuousState_.currentSines[axis])
                sum += s.amplitude * std::sin(2.0 * M_PI * s.frequency * t + s.phase);
            result[axis][i] = (num_sines > 0) ? sum : 0.0;
        }

        // Normalize so peak magnitude == max_amplitude
        if (num_sines > 0) {
            double peak = 0.0;
            for (double v : result[axis])
                peak = std::max(peak, std::abs(v));
            if (peak > 1e-9) {
                double scale = params.max_amplitude / peak;
                for (double& v : result[axis])
                    v *= scale;
            }
        }
    }

    std::cout << "Generated sum-of-sinusoids VFF: X="
        << continuousState_.currentSines[0].size() << " sines, Y="
        << continuousState_.currentSines[1].size() << " sines, Z="
        << continuousState_.currentSines[2].size() << " sines" << std::endl;

    return result;
}

// ============================================================================
// Helpers
// ============================================================================

// Amplitude: uniform draw from [0, max_amplitude], rounded to nearest 0.1,
// then a random sign applied.
double VffGenerator::drawAmplitude(const VffParams& params) {
    std::uniform_real_distribution<double> amp_dist(0.0, params.max_amplitude);
    double raw = amp_dist(rng_);
    double mag = std::round(raw * 10.0) / 10.0;  // round to nearest 0.1
    mag = std::clamp(mag, 0.0, params.max_amplitude);

    std::uniform_int_distribution<int> sign_dist(0, 1);
    double sign = sign_dist(rng_) ? 1.0 : -1.0;
    return mag * sign;
}

// Dwell: exponential distribution with mean ~400 samples, hard floor of 50.
int VffGenerator::drawDwell(const VffParams& params) {
    std::exponential_distribution<double> exp_dist(1.0 / params.mean_dwell_samples);
    int d = static_cast<int>(std::round(exp_dist(rng_)));
    return std::max(d, params.min_dwell_samples);
}

// Control point for quadratic Bezier.
// straight == true  -> P1 = midpoint (straight line)
// straight == false -> P1 drawn uniformly from the range [A,B] extended by
//                      +/- max_amplitude on each side, giving genuine curvature.
double VffGenerator::drawControlPoint(
    double A, double B, const VffParams& params, bool straight)
{
    if (straight) {
        return (A + B) / 2.0;
    }

    double lo = std::clamp(std::min(A, B) - params.max_amplitude, -params.max_amplitude, params.max_amplitude);
    double hi = std::clamp(std::max(A, B) + params.max_amplitude, -params.max_amplitude, params.max_amplitude);
    if (lo >= hi) return (A + B) / 2.0;  // fallback to straight if range collapses
    std::uniform_real_distribution<double> cp_dist(lo, hi);
    return cp_dist(rng_);
}