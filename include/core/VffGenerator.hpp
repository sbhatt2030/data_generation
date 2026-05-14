#pragma once
#include <random>
#include <vector>
#include <array>

#include <Eigen/Dense>
#include "core/input_data_point.hpp"


enum class VffType {
    NO_VFF = 0,
    SQUARE_WAVE = 1,
    SMOOTH_RAMP = 2,
    SUM_OF_SINUSOIDS = 3,
    EXISTING_SEQUENCE = 4
};

class VffGenerator {
public:
    struct VffParams {
        // Amplitude: drawn from [0, max_amplitude] rounded to nearest 0.1, random sign
        double max_amplitude = 5.0;   // CSV: vff_max_amplitude

        // Dwell: exponential distribution
        double mean_dwell_samples = 500.0; // CSV: vff_mean_dwell
        int    min_dwell_samples = 80;    // CSV: vff_min_dwell

        // Sum-of-sinusoids parameters (Type 3 only)
        double min_frequency = 0.1;   // CSV: vff_min_freq
        double max_frequency = 10.0;  // CSV: vff_max_freq
        int    min_num_sines = 2;     // CSV: vff_min_sines
        int    max_num_sines = 8;     // CSV: vff_max_sines
    };

    // Per-sinusoid state for Type 3
    struct SineComponent {
        double amplitude;
        double frequency;
        double phase;
    };

    // State carried across chunks
    struct ContinuousState {
        bool initialized = false;
        int  chunkCounter = 0;

        // Types 1 & 2
        std::array<double, 3> current_amplitude = { 0.0, 0.0, 0.0 };
        std::array<int, 3> dwell_remaining = { 0,   0,   0 };

        // Type 2: two-phase alternation per axis
        enum class Phase { RAMP, FLAT };
        std::array<Phase, 3> phase = { Phase::FLAT, Phase::FLAT, Phase::FLAT };
        std::array<double, 3> next_amplitude = { 0.0, 0.0, 0.0 };
        std::array<double, 3> control_point = { 0.0, 0.0, 0.0 };
        std::array<int, 3> segment_length = { 0,   0,   0 };

        // Type 3: active sine components per axis
        std::array<std::vector<SineComponent>, 3> currentSines;
    };

    explicit VffGenerator(unsigned int seed = 0);

    // Primary interface
    void addVffToChunk(
        std::vector<InputDataPoint>& chunk,
        VffType vffType,
        const VffParams& params,
        double dt);

    // Raw generation (exposed for testing)
    std::array<std::vector<double>, 3> generateVffChunk(
        int chunkSize,
        VffType vffType,
        const VffParams& params,
        double dt);

    void resetContinuousState();

private:
    mutable std::mt19937 rng_;
    ContinuousState continuousState_;

    std::array<std::vector<double>, 3> generateSquareWave(
        int chunkSize, const VffParams& params);

    std::array<std::vector<double>, 3> generateSmoothRamp(
        int chunkSize, const VffParams& params);

    std::array<std::vector<double>, 3> generateSumOfSinusoids(
        int chunkSize, const VffParams& params, double dt);

    // Draw amplitude: uniform in [0, max], rounded to 0.1, random sign
    double drawAmplitude(const VffParams& params);

    // Draw dwell: exponential(mean=400), floor 50
    int    drawDwell(const VffParams& params);

    // Draw quadratic Bezier control point between A and B
    double drawControlPoint(double A, double B, const VffParams& params, bool straight);
};