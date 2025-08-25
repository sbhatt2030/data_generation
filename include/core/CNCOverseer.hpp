#pragma once

#include "core/CNCExperimentRunner.hpp"
#include "HurcoConnection.hpp"
#include "CSVParser.hpp"
#include <string>
#include <vector>
#include <memory>
#include <fstream>

/**
 * Batch execution results
 */
struct BatchResult {
    size_t totalExperiments = 0;
    size_t successfulExperiments = 0;
    size_t failedExperiments = 0;
    std::vector<std::string> failedExperimentIds;
    double totalExecutionTimeSeconds = 0.0;
    std::string batchStartTime;
    std::string batchEndTime;
    std::string errorLogPath;
};

/**
 * CNC Overseer - Main batch orchestrator for CNC experiments
 *
 * Coordinates between:
 * - CSV experiment configurations
 * - CNC machine control (via HurcoConnection)
 * - Data collection (via CNCExperimentRunner)
 *
 * Single-threaded design for simplicity and deterministic execution
 */
class CNCOverseer {
public:
    /**
     * Constructor
     */
    CNCOverseer();

    /**
     * Destructor - ensures clean shutdown
     */
    ~CNCOverseer();

    /**
     * Load system configuration
     * @param systemConfigPath Path to system configuration JSON
     * @return true if loaded successfully
     */
    bool loadSystemConfiguration(const std::string& systemConfigPath);

    /**
     * Connect to CNC machine
     * @return true if connection successful
     */
    bool connectToCNC();

    /**
     * Disconnect from CNC machine
     * @return true if disconnect successful
     */
    bool disconnectFromCNC();

    /**
     * Run batch of experiments from CSV file
     * @param csvFilePath Path to CSV experiment configuration file
     * @param outputBaseDir Base output directory for all experiments
     * @return BatchResult with execution statistics
     */
    BatchResult runExperimentBatch(const std::string& csvFilePath, const std::string& outputBaseDir = "./batch_output");

    /**
     * Run single experiment with CNC coordination
     * @param config Experiment configuration
     * @return true if experiment completed successfully
     */
    bool runSingleExperiment(const ExperimentConfig& config);

    /**
     * Check if CNC program is complete (for ExperimentRunner callback)
     * @return true if CNC program has finished execution
     */
    bool isCNCProgramComplete();

    /**
     * Get current CNC program status
     * @return Current ProgramStatus
     */
    ProgramStatus getCurrentCNCStatus();

    /**
     * Get last error message
     * @return Error message
     */
    const std::string& getLastError() const { return lastError_; }

    /**
     * Get batch execution statistics
     * @return Current batch result
     */
    const BatchResult& getBatchResult() const { return batchResult_; }

private:
    // Core components
    //std::unique_ptr<CNCExperimentRunner> experimentRunner_;
    std::unique_ptr<HurcoConnection> hurcoConnection_;

    // System state
    bool systemConfigured_ = false;
    bool cncConnected_ = false;
    std::string systemConfigPath_;

    // Experiment coordination
    std::atomic<bool> experimentComplete_{ false };
    ProgramStatus currentProgramStatus_ = ProgramStatus::UNKNOWN;

    // Batch execution state
    BatchResult batchResult_;
    std::string currentExperimentId_;
    std::chrono::steady_clock::time_point batchStartTime_;
    std::ofstream errorLog_;

    // Error handling
    std::string lastError_;

    /**
     * Initialize experiment runner with system configuration
     * @return true if initialization successful
     */
    //bool initializeExperimentRunner();

    /**
     * Execute single experiment workflow
     * @param config Experiment configuration
     * @return true if successful
     */
    bool executeSingleExperimentWorkflow(const ExperimentConfig& config);

    /**
     * Create unique output directory for experiment
     * @param config Experiment configuration
     * @param baseDir Base output directory
     * @return Full path to experiment output directory
     */
    bool shouldTerminateExperiment() const;
    void onCNCStatusChange(ProgramStatus oldStatus, ProgramStatus newStatus);

    std::string createExperimentOutputDirectory(const ExperimentConfig& config, const std::string& baseDir);

    /**
     * Setup error logging for batch
     * @param outputBaseDir Base output directory
     * @return true if setup successful
     */
    bool setupBatchLogging(const std::string& outputBaseDir);

    /**
     * Log experiment failure
     * @param experimentId Experiment ID that failed
     * @param error Error message
     */
    void logExperimentFailure(const std::string& experimentId, const std::string& error);

    /**
     * Update batch statistics
     * @param experimentId Experiment ID
     * @param success Whether experiment succeeded
     */
    void updateBatchStatistics(const std::string& experimentId, bool success);

    /**
     * Print batch summary
     */
    void printBatchSummary();

    /**
     * External termination callback for ExperimentRunner
     * @return true if experiment should terminate (CNC program complete)
     */
    bool experimentTerminationCallback();

    /**
     * Set error message
     * @param error Error message
     */
    void setError(const std::string& error);

    /**
     * Get current timestamp string
     * @return Formatted timestamp
     */
    std::string getCurrentTimestamp() const;
};