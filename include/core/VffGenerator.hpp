#pragma once
#include <random>
#include <vector>
#include <array>

#include <Eigen/Dense>
#include "core/input_data_point.hpp"


enum class VffType {
    SMOOTH_GAUSSIAN = 0,
    SMOOTH_GAUSSIAN_DC_SHIFT = 1,
    SPARSE_VFF = 2,
    NO_VFF = 3
};

class VffGenerator {
public:
    /**
     * Clean VFF parameters - maps directly to CSV columns
     */
    struct VffParams {
        double min_dc_shift = -5.0;         // CSV: vff_min_dc
        double max_dc_shift = 5.0;          // CSV: vff_max_dc
        double max_amplitude = 10.0;        // CSV: vff_max_amplitude
        double max_frequency = 50.0;        // CSV: vff_max_freq
        double sparse_probability = 0.02;   // CSV: vff_sparse_prob
    };

    /**
     * Constructor
     */
    explicit VffGenerator(unsigned int seed = 0);

    /**
     * Generate VFF signals for a chunk of samples (8000 points)
     * Each chunk starts fresh - no continuity between chunks
     */
    std::array<std::vector<double>, 3> generateVffChunk(
        int chunkSize,
        VffType vffType,
        const VffParams& params,
        double dt);

    /**
     * Add VFF signals to existing InputDataPoint chunk
     */
    void addVffToChunk(
        std::vector<InputDataPoint>& chunk,
        VffType vffType,
        const VffParams& params,
        double dt);

    /**
     * Reset continuous generation state (for new experiments)
     */
    void resetContinuousState();

    /**
     * Legacy interface methods (keep for compatibility)
     */
    std::array<std::vector<double>, 3> generateAllAxes();
    std::array<std::vector<double>, 3> generateAllAxes(
        const std::array<double, 3>& amplitudes,
        const std::array<double, 3>& alphas);

private:
    mutable std::mt19937 rng_;

    // Continuous generation state (minimal - chunks are independent)
    struct ContinuousState {
        bool initialized = false;
        int chunkCounter = 0;
    } continuousState_;

    // Butterworth filter state for smooth Gaussian VFF
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

    // Type-specific generation methods
    std::array<std::vector<double>, 3> generateSmoothGaussian(
        int chunkSize, const VffParams& params, double dt);

    std::array<std::vector<double>, 3> generateSmoothGaussianDCShift(
        int chunkSize, const VffParams& params, double dt);

    std::array<std::vector<double>, 3> generateSparseVff(
        int chunkSize, const VffParams& params, double dt);

    // Filtering methods
    void applyButterworthFilterSingleAxis(double& sample, int axis, double cutoff_freq, double dt);

    // Utility methods
    double generateGaussianNoise(double mean, double std_dev) const;

    // Legacy methods (simplified - keep for compatibility)
    std::vector<double> generateType1(double amplitude, double alpha) const;
    std::vector<double> generateType2(double amplitude, double alpha) const;
    std::vector<double> generateType3(double amplitude) const;
    double getRandomAmplitude() const;
    double getRandomAlpha() const;
    std::vector<double> generateSmoothedNoise(double alpha) const;

    std::array<std::vector<double>, 3> generateVffTestPattern(int chunkSize, const VffParams& params, double dt);

    // Legacy configuration (for compatibility)
    double minAmplitude_ = 0.1;
    double maxAmplitude_ = 10.0;
    double minAlpha_ = 0.01;
    double maxAlpha_ = 0.2;
    int signalLength_ = 8000;
    double sparseProbability_ = 0.02;
};