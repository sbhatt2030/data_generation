#pragma once

#include "core/GenerationPipeline.hpp"
#include "core/InjectionPipeline.hpp"
#include "core/ExtractionPipeline.hpp"
#include "core/MotionService.h"
#include "core/gcode_generator.hpp"
#include "core/KinematicNoiseGenerator.hpp"
#include "core/system_constants.hpp"
#include "core/HurcoConnection.hpp"
#include <memory>
#include <chrono>
#include <string>
#include <optional>
#include <random>
#include <functional>  // NEW: For external termination callback

/**
 * Flexible seed configuration for experiments
 */
struct SeedConfiguration {
    // Option 1: Master seed approach (generates all others)
    std::optional<unsigned int> masterSeed;

    // Option 2: Individual seed specification (overrides master seed)
    std::optional<unsigned int> gcodeGeneratorSeed;
    std::optional<unsigned int> noiseGeneratorSeed;
    std::optional<unsigned int> vffGeneratorSeed;

    // Utility methods
    bool usesMasterSeed() const { return masterSeed.has_value(); }
    bool usesIndividualSeeds() const {
        return gcodeGeneratorSeed.has_value() ||
            noiseGeneratorSeed.has_value() ||
            vffGeneratorSeed.has_value();
    }
};

/**
 * System configuration loaded from JSON file
 */
struct SystemConfiguration {
    // Machine constraints
    MachineConstraints machineConstraints;

    // Motion configuration
    MotionConfig motionConfig;

    // System timing
    struct {
        double mainLoopFrequency = SystemConstants::Timing::MAIN_LOOP_FREQUENCY_HZ;
        double loggingFrequency = SystemConstants::Timing::LOGGING_FREQUENCY_HZ; 
        long smrWriteTimeout = 0;              // Non-blocking
        long smrReadTimeout = 0;               // Non-blocking
    } systemTiming;

    // Buffer configuration
    struct {
        size_t injectionBufferCapacity = SystemConstants::Buffers::INJECTION_BUFFER_CAPACITY;
        size_t injectionRefillTrigger = SystemConstants::Buffers::INJECTION_REFILL_TRIGGER;
        size_t extractionBufferCapacity = SystemConstants::Buffers::EXTRACTION_BUFFER_SIZE;
        size_t smrBufferSize = SystemConstants::Buffers::SMR_SHARED_MEMORY_SIZE;
        size_t loggingInterval = 1000;
    } bufferConfig;

    // Motion detection thresholds
    struct {
        double globalVelocityThreshold = 1e-6;
        double globalAccelerationThreshold = 1e-3;
        double axisVelocityThreshold = 1e-4;
        double axisAccelerationThreshold = 1e-2;
    } motionThresholds;

};

/**
 * Experiment-specific parameters
 */
struct ExperimentConfig {
    // Output and session management
    std::string experimentId;
    std::string familyId;
    std::string outputDirectory;
    SeedConfiguration seedConfig;

    // G-code generation parameters
    GenerationParams gcodeParams;
    std::string existingGcodeFile;        // Optional: use existing file instead of generating

    // Noise generation parameters
    KinematicNoiseGenerator::NoiseParams noiseParams;
    KinematicNoiseType noiseType = KinematicNoiseType::SMOOTH_GAUSSIAN_BANDPASS;

    // VFF generation parameters
    VffConfig vffConfig;

    // Runtime experiment options
    bool enableDetailedLogging = false;

    // Default constructor with sensible defaults
    ExperimentConfig();
};

/**
 * Result of a completed experiment
 */
struct ExperimentResult {
    bool success = false;
    std::string sessionFolder;
    size_t totalTrajectoriesGenerated = 0;
    size_t totalPointsCollected = 0;
    size_t csvFilesWritten = 0;
    double executionTimeSeconds = 0.0;
    std::string errorMessage;

    // Actual seeds used (for reproduction)
    unsigned int actualGcodeGeneratorSeed = 0;
    unsigned int actualNoiseGeneratorSeed = 0;
    unsigned int actualVffGeneratorSeed = 0;

    // Final statistics from pipelines
    InjectionStats finalInjectionStats;
    ExtractionStats finalExtractionStats;

    // Aggregate system health metrics
    double avgWriteSuccessRate = 0.0;
    double avgReadSuccessRate = 0.0;
    size_t totalWriteFailures = 0;
    size_t totalReadFailures = 0;
};

/**
 * Main controller for executing complete CNC data collection experiments
 * NEW: Enhanced with external control interface for Overseer orchestration
 */
class CNCExperimentRunner {
public:
    /**
     * Constructor
     */
    CNCExperimentRunner();

    /**
     * Destructor - ensures clean shutdown
     */
    ~CNCExperimentRunner();

    /**
     * Load system configuration from JSON file
     * Called once at startup, before running experiments
     * @param configFile Path to system configuration JSON file
     * @return true if configuration loaded successfully
     */
    bool loadSystemConfiguration(const std::string& configFile);

    /**
     * Run a complete experiment with specified parameters
     * @param experimentConfig Experiment-specific parameters
     * @return true if experiment completed successfully
     */
    //bool runExperiment(const ExperimentConfig& experimentConfig);


    /**
     * NEW: Get the generated G-code file path for CNC machine
     * @return Full path to generated G-code file
     */
    std::string getGeneratedGCodePath() const;


    /**
     * Get result of the last completed experiment
     * @return Experiment result with statistics and status
     */
    ExperimentResult getResult() const { return lastResult_; }

    /**
     * Request graceful shutdown of current experiment
     * Thread-safe - can be called from signal handlers
     */
    void requestStop();

    /**
     * Check if system configuration has been loaded
     * @return true if loadSystemConfiguration was successful
     */
    bool isConfigurationLoaded() const { return configurationLoaded_; }

    /**
 * NEW: Initialize experiment and generate G-code file without starting main loop
 * This allows the Overseer to get the G-code path before starting data collection
 * @param experimentConfig Experiment-specific parameters
 * @return true if initialization and G-code generation successful
 */
    bool initializeExperiment(const ExperimentConfig& experimentConfig);

    /**
     * NEW: Start the main experiment loop (call after initializeExperiment)
     * @return true if experiment completed successfully
     */
    bool startExperimentLoop();
    void logAggregatedSystemStatusWithCNC(ProgramStatus cncStatus, bool completionDetected);
    /**
     * Set CNC connection for direct status polling
     * @param connection HurcoConnection instance
     */
    void setCNCConnection(HurcoConnection* connection) {
        cncConnection_ = connection;
    }

private:
    // System configuration
    SystemConfiguration systemConfig_;
    std::string experimentLogFile_;
    std::string systemStatusFile_;
    bool configurationLoaded_ = false;

    // Pipeline components
    std::unique_ptr<MotionService> motionService_;
    std::unique_ptr<GenerationPipeline> generationPipeline_;
    std::unique_ptr<InjectionPipeline> injectionPipeline_;
    std::unique_ptr<ExtractionPipeline> extractionPipeline_;

    // Experiment state
    ExperimentResult lastResult_;
    std::atomic<bool> stopRequested_{ false };
    std::chrono::steady_clock::time_point experimentStartTime_;
    HurcoConnection* cncConnection_ = nullptr;


    // Logging state
    int mainLoopCounter_ = 0;
    int loggingInterval_ = 200;  // Log every 2 seconds at 100Hz

    // Resolved seeds (for result reporting)
    unsigned int resolvedGcodeGeneratorSeed_ = 0;
    unsigned int resolvedNoiseGeneratorSeed_ = 0;
    unsigned int resolvedVffGeneratorSeed_ = 0;

    // Initialization methods
    bool initializeMotionService();
    bool initializePipelines(const ExperimentConfig& experimentConfig);
    SeedConfiguration resolveSeeds(const SeedConfiguration& seedConfig);

    // Main execution methods
    bool executeMainLoop();
    void performSingleCycle();
    bool shouldContinueExperiment();  // MODIFIED: Now checks external termination

    // Monitoring and logging
    void initializeLogging();
    void logAggregatedSystemStatus();
    void updateExperimentResult();
    void logSeedConfiguration(const SeedConfiguration& seedConfig);
    std::string getLogsDirectory() const;

    // Shutdown and cleanup
    void performFinalCleanup();
    void shutdownPipelines();
    void shutdownMotionService();

    // Utility methods
    std::string getCurrentTimestamp() const;
    double getElapsedTimeSeconds() const;

    // Error handling
    bool handleComponentError(const std::string& component, const std::string& error);
    void setExperimentError(const std::string& errorMessage);
};