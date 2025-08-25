#pragma once

#include "core/KinematicNoiseGenerator.hpp"
#include "core/VffGenerator.hpp"
#include "core/gcode_generator.hpp"
#include "core/motion_config.hpp"
#include "core/trajectory_types.hpp"
#include "core/system_constants.hpp"
#include <vector>
#include <string>
#include <memory>
#include <set>

/**
 * Input data point structure for pipeline communication
 */
//struct InputDataPoint {
//    double dev_x = 0.0, dev_y = 0.0, dev_z = 0.0;  // Position deviations
//    double vff_x = 0.0, vff_y = 0.0, vff_z = 0.0;  // VFF signals  
//    int line_number = -1;                           // G-code line reference
//};

/**
 * VFF configuration
 */
struct VffParams {
    double max_amplitude = 10.0;
    double dc_shift_min = -5.0;
    double dc_shift_max = 15.0;
    double sparse_probability = 0.02;
    double min_frequency = 0.1;
    double max_frequency = 25.0;
    double smoothness_factor = 0.9;
};



struct VffConfig {
    bool useVffGenerator = true;
    VffType vffType = VffType::NO_VFF;

    // NEW: Add VffParams structure
    struct VffParams {
        double min_dc_shift = -5.0;
        double max_dc_shift = 5.0;
        double max_amplitude = 10.0;
        double max_frequency = 50.0;
        double sparse_probability = 0.02;
    } vffParams;

    // Legacy fields (keep for compatibility)
    double minAmplitude = 0.1;
    double maxAmplitude = 10.0;
    double minAlpha = 0.01;
    double maxAlpha = 0.2;
    bool usePerAxisVff = false;
    double sparseVffProbability = 0.02;
    double minSparseVffAmplitude = 1.0;
    double maxSparseVffAmplitude = 25.0;
    std::array<double, 3> fixedAmplitudes = { 0.0, 0.0, 0.0 };
    std::array<double, 3> fixedAlphas = { 0.0, 0.0, 0.0 };
};

/**
 * Simplified Generation Pipeline - Pure continuous noise generation
 * No G-code parsing - Overseer handles trajectory execution via CNC REST API
 */
class GenerationPipeline {
public:
    /**
     * Constructor
     * @param outputFolder Base folder for session data
     * @param masterSeed Master seed for reproducible random generation
     */
    GenerationPipeline(const std::string& outputFolder,
        unsigned int gcodeGeneratorSeed,
        unsigned int noiseGeneratorSeed,
        unsigned int vffGeneratorSeed);

    /**
     * Initialize for continuous noise generation
     * @param gcodeFile Optional G-code file (for session naming only)
     * @param genParams Parameters for G-code generation (if needed)
     * @return true if initialization successful
     */
    bool initialize(const std::string& gcodeFile = "",
        const GenerationParams& genParams = GenerationParams{});

    /**
     * Enable continuous noise generation mode
     * @param enable True to enable continuous mode (always true in this version)
     */
    void enableContinuousGeneration(bool enable = true);

    /**
     * Check if in continuous generation mode
     * @return true (always in continuous mode)
     */
    bool isContinuousMode() const { return true; }

    /**
     * Check if more commands available
     * @return true (always true in continuous mode)
     */
    bool hasMoreCommands() const { return true; }

    /**
     * Generate next chunk of noise data
     * @return Vector of noise data points (8000 samples)
     */
    std::vector<InputDataPoint> generateNextCommandData();

    /**
     * Get remaining command count
     * @return SIZE_MAX (infinite in continuous mode)
     */
    size_t getRemainingCommands() const { return SIZE_MAX; }

    /**
     * Get dummy line numbers for extraction pipeline
     * @return Set with single dummy line number
     */
    std::set<int> getAllLineNumbers() const;

    // Configuration setters
    void setVffConfig(const VffConfig& config);
    void setNoiseParams(const KinematicNoiseGenerator::NoiseParams& params);
    void setMachineConstraints(const MachineConstraints& constraints);
    void setMotionConfig(const MotionConfig& config);
    void setNoiseType(KinematicNoiseType type);

    // Status getters
    const std::string& getSessionFolder() const { return uniqueSessionFolder_; }
    const std::string& getGCodeFilePath() const { return gcodeFilePath_; }
    const VffConfig& getVffConfig() const { return vffConfig_; }

private:
    // Session management
    std::string baseOutputFolder_;
    std::string uniqueSessionFolder_;
    std::string gcodeFilePath_;
    unsigned int masterSeed_;

    // Noise generation components
    std::unique_ptr<KinematicNoiseGenerator> noiseGenerator_;
    std::unique_ptr<VffGenerator> vffGenerator_;

    // Configuration
    KinematicNoiseGenerator::NoiseParams noiseParams_;
    MachineConstraints machineConstraints_;
    MotionConfig motionConfig_;
    KinematicNoiseType noiseType_;
    VffConfig vffConfig_;

    // Continuous generation state
    int continuousChunkCounter_ = 0;

    // Seeds for reproducible generation
    unsigned int gcodeGeneratorSeed_;
    unsigned int noiseGeneratorSeed_;
    unsigned int vffGeneratorSeed_;

    // Internal methods
    void initializeGenerators();
    bool createUniqueSessionFolder();
    bool copyGCodeFile(const std::string& sourceFile);
    bool generateGCodeFile(const GenerationParams& genParams);
    bool saveConfigurationFiles(const GenerationParams& genParams, bool usedExistingGCode);
    void saveVffConfigToFile() const;
    void saveNoiseConfigToFile() const;
    void saveMachineConfigToFile() const;
    std::string getCurrentTimestamp() const;

    // Noise generation methods
    std::vector<InputDataPoint> generateContinuousNoiseChunk();
    void addVFFToChunk(std::vector<InputDataPoint>& chunk);
};