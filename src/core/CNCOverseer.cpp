#include "core/CNCOverseer.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;

CNCOverseer::CNCOverseer() {
    hurcoConnection_ = std::make_unique<HurcoConnection>();
    experimentRunner_ = std::make_unique<CNCExperimentRunner>();
}

CNCOverseer::~CNCOverseer() {
    if (cncConnected_) {
        disconnectFromCNC();
    }

    if (errorLog_.is_open()) {
        errorLog_.close();
    }
}

bool CNCOverseer::loadSystemConfiguration(const std::string& systemConfigPath) {
    systemConfigPath_ = systemConfigPath;

    std::cout << "Loading system configuration: " << systemConfigPath_ << std::endl;

    std::cout << "Step 1: Initializing fresh experiment runner..." << std::endl;
    if (!experimentRunner_->loadSystemConfiguration(systemConfigPath_)) {
        setError("Failed to load system configuration into experiment runner");
        return false;
    }

    experimentRunner_->setCNCConnection(hurcoConnection_.get());

    if (!std::filesystem::exists(systemConfigPath_)) {
        setError("System configuration file not found: " + systemConfigPath_);
        systemConfigured_ = true;
        return true;
    }

    systemConfigured_ = true;
    std::cout << "System configuration loaded successfully" << std::endl;
    return true;
}

bool CNCOverseer::connectToCNC() {
    std::cout << "Connecting to Hurco CNC machine..." << std::endl;

    if (!hurcoConnection_->startConnection()) {
        setError("Failed to connect to CNC: " + hurcoConnection_->getLastError());
        return false;
    }

    cncConnected_ = true;
    std::cout << "CNC connection established successfully" << std::endl;
    return true;
}

bool CNCOverseer::disconnectFromCNC() {
    if (!cncConnected_) {
        return true;
    }

    std::cout << "Disconnecting from CNC..." << std::endl;

    hurcoConnection_->stopConnection();
    cncConnected_ = false;

    std::cout << "CNC disconnection successful" << std::endl;
    return true;
}

BatchResult CNCOverseer::runExperimentBatch(const std::string& csvFilePath, const std::string& outputBaseDir) {
    if (!systemConfigured_) {
        setError("System configuration not loaded");
        return batchResult_;
    }

    if (!cncConnected_) {
        setError("CNC not connected");
        return batchResult_;
    }

    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STARTING CNC EXPERIMENT BATCH" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    std::cout << "CSV file: " << csvFilePath << std::endl;
    std::cout << "Output directory: " << outputBaseDir << std::endl;

    // Initialize batch state
    batchStartTime_ = std::chrono::steady_clock::now();
    batchResult_ = BatchResult{};
    batchResult_.batchStartTime = getCurrentTimestamp();

    // Setup logging
    if (!setupBatchLogging(outputBaseDir)) {
        setError("Failed to setup batch logging");
        return batchResult_;
    }

    // Parse experiments from CSV
    std::vector<ExperimentConfig> experiments = CSVParser::parseCSV(csvFilePath);
    if (experiments.empty()) {
        setError("No valid experiments found in CSV: " + CSVParser::getLastError());
        return batchResult_;
    }

    batchResult_.totalExperiments = experiments.size();
    std::cout << "Loaded " << experiments.size() << " experiments from CSV" << std::endl;

    // Execute experiments sequentially
    for (size_t i = 0; i < experiments.size(); ++i) {
        ExperimentConfig& config = experiments[i];

        std::cout << "\n" << std::string(60, '-') << std::endl;
        std::cout << "EXPERIMENT " << (i + 1) << "/" << experiments.size()
            << ": " << config.experimentId << std::endl;
        std::cout << std::string(60, '-') << std::endl;

        // Create unique output directory for this experiment
        config.outputDirectory = createExperimentOutputDirectory(config, outputBaseDir);

        // Execute experiment
        currentExperimentId_ = config.experimentId;
        bool success = runSingleExperiment(config);

        // Update statistics
        updateBatchStatistics(config.experimentId, success);

        if (!success) {
            std::cout << "❌ Experiment " << config.experimentId << " FAILED: " << lastError_ << std::endl;
            logExperimentFailure(config.experimentId, lastError_);
        }
        else {
            std::cout << "✅ Experiment " << config.experimentId << " COMPLETED SUCCESSFULLY" << std::endl;
        }
    }

    // Finalize batch
    auto batchEndTime = std::chrono::steady_clock::now();
    batchResult_.totalExecutionTimeSeconds =
        std::chrono::duration<double>(batchEndTime - batchStartTime_).count();
    batchResult_.batchEndTime = getCurrentTimestamp();

    printBatchSummary();

    return batchResult_;
}

bool CNCOverseer::runSingleExperiment(const ExperimentConfig& config) {
    if (!systemConfigured_ || !cncConnected_) {
        setError("System not properly configured or CNC not connected");
        return false;
    }

    try {
        return executeSingleExperimentWorkflow(config);
    }
    catch (const std::exception& e) {
        setError("Exception during experiment: " + std::string(e.what()));
        return false;
    }
}

bool CNCOverseer::executeSingleExperimentWorkflow(const ExperimentConfig& config) {
    std::cout << "Starting experiment workflow for: " << config.experimentId << std::endl;

    // Reset experiment state
    experimentComplete_ = false;
    currentProgramStatus_ = ProgramStatus::UNKNOWN;


    // **CRITICAL: Clear SMR BEFORE initializing experiment** ⭐
    std::cout << "Step 1.5: Clearing SMR buffers from previous experiment..." << std::endl;
    // Force creation of MotionService just to clear buffers
    {
        MotionService tempMotionService;
        int errorCode = 0;
        // tempMotionService destructor handles cleanup
    }

    // Give RT system time to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initialize experiment (generates G-code and prepares everything)
    std::cout << "Step 2: Initializing experiment..." << std::endl;
    if (!experimentRunner_->initializeExperiment(config)) {
        setError("Failed to initialize experiment");
        return false;
    }

    // Get G-code file path
    std::string gcodeFilePath = experimentRunner_->getGeneratedGCodePath();
    if (gcodeFilePath.empty()) {
        setError("No G-code file generated");
        return false;
    }
    std::cout << "Step 3: G-code generated: " << gcodeFilePath << std::endl;

    // Load and run CNC program
    std::cout << "Step 4: Starting CNC program..." << std::endl;
    if (!hurcoConnection_->loadAndRunProgram(gcodeFilePath)) {
        setError("Failed to send load/run command: " + hurcoConnection_->getLastError());
        return false;
    }


    // Start data collection (this will block until CNC completes)
    std::cout << "Step 5: Starting data collection..." << std::endl;
    if (!experimentRunner_->startExperimentLoop()) {
        setError("Failed to start data collection: " + experimentRunner_->getResult().errorMessage);
        return false;
    }

    // Experiment is complete when data collection finishes
    auto finalResult = experimentRunner_->getResult();
    if (!finalResult.success) {
        setError("Experiment failed: " + finalResult.errorMessage);
        return false;
    }

    std::cout << "Experiment completed successfully:" << std::endl;
    std::cout << "  Session folder: " << finalResult.sessionFolder << std::endl;
    std::cout << "  CSV files written: " << finalResult.csvFilesWritten << std::endl;
    std::cout << "  Execution time: " << finalResult.executionTimeSeconds << "s" << std::endl;

    // Log position spikes to batch error log if any occurred
        if (finalResult.finalExtractionStats.positionSpikeCount > 0 && errorLog_.is_open()) {
            errorLog_ << getCurrentTimestamp() << " - EXPERIMENT " << config.experimentId
                << ": " << finalResult.finalExtractionStats.positionSpikeCount
                << " position spikes detected (see position_spikes.log)" << std::endl;
            errorLog_.flush();
        }

    hurcoConnection_->resetStatusForNewExperiment();
    // experimentRunner will be automatically destroyed here (unique_ptr)
    return true;
}

bool CNCOverseer::isCNCProgramComplete() {
    if (!cncConnected_) {
        return true; // If not connected, consider it "complete" to avoid hanging
    }

    return hurcoConnection_->isProgramComplete();
}

ProgramStatus CNCOverseer::getCurrentCNCStatus() {
    if (!cncConnected_) {
        return ProgramStatus::UNKNOWN;
    }

    return hurcoConnection_->getCurrentProgramStatus();
}

//bool CNCOverseer::experimentTerminationCallback() {
//    if (!cncConnected_) {
//        return true; // If not connected, consider it "complete" to avoid hanging
//    }
//    // Simple: check if CNC program is complete
//    return isCNCProgramComplete();
//}

// Private helper methods

std::string CNCOverseer::createExperimentOutputDirectory(const ExperimentConfig& config, const std::string& baseDir) {
    // Create directory structure: baseDir/family_id/experiment_id_timestamp/
    fs::path experimentPath = fs::path(baseDir) / config.familyId /
        (config.experimentId + "_" + getCurrentTimestamp());

    try {
        fs::create_directories(experimentPath);
        return experimentPath.string();
    }
    catch (const std::exception& e) {
        std::cerr << "Warning: Failed to create experiment directory: " << e.what() << std::endl;
        return baseDir; // Fallback to base directory
    }
}

bool CNCOverseer::setupBatchLogging(const std::string& outputBaseDir) {
    try {
        fs::create_directories(outputBaseDir);

        std::string errorLogPath = (fs::path(outputBaseDir) / "batch_errors.log").string();
        batchResult_.errorLogPath = errorLogPath;

        errorLog_.open(errorLogPath, std::ios::app);
        if (!errorLog_.is_open()) {
            setError("Failed to open error log file: " + errorLogPath);
            return false;
        }

        errorLog_ << "\n=== BATCH STARTED: " << getCurrentTimestamp() << " ===" << std::endl;
        return true;

    }
    catch (const std::exception& e) {
        setError("Exception setting up batch logging: " + std::string(e.what()));
        return false;
    }
}

void CNCOverseer::logExperimentFailure(const std::string& experimentId, const std::string& error) {
    if (errorLog_.is_open()) {
        errorLog_ << getCurrentTimestamp() << " - FAILED: " << experimentId
            << " - " << error << std::endl;
        errorLog_.flush();
    }

    batchResult_.failedExperimentIds.push_back(experimentId);
}

void CNCOverseer::updateBatchStatistics(const std::string& experimentId, bool success) {
    if (success) {
        batchResult_.successfulExperiments++;
    }
    else {
        batchResult_.failedExperiments++;
    }
}

void CNCOverseer::printBatchSummary() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "BATCH EXECUTION SUMMARY" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    std::cout << "Total experiments: " << batchResult_.totalExperiments << std::endl;
    std::cout << "Successful: " << batchResult_.successfulExperiments << " ✅" << std::endl;
    std::cout << "Failed: " << batchResult_.failedExperiments << " ❌" << std::endl;
    std::cout << "Success rate: " << std::fixed << std::setprecision(1)
        << (100.0 * batchResult_.successfulExperiments / batchResult_.totalExperiments) << "%" << std::endl;
    std::cout << "Total execution time: " << std::fixed << std::setprecision(1)
        << batchResult_.totalExecutionTimeSeconds << " seconds" << std::endl;

    if (!batchResult_.failedExperimentIds.empty()) {
        std::cout << "\nFailed experiments:" << std::endl;
        for (const auto& failedId : batchResult_.failedExperimentIds) {
            std::cout << "  - " << failedId << std::endl;
        }
        std::cout << "See error log: " << batchResult_.errorLogPath << std::endl;
    }

    std::cout << std::string(80, '=') << std::endl;
}

void CNCOverseer::setError(const std::string& error) {
    lastError_ = error;
    std::cerr << "CNCOverseer Error: " << error << std::endl;
}

std::string CNCOverseer::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    std::tm local_time;
    localtime_s(&local_time, &time_t);
    ss << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

bool CNCOverseer::shouldTerminateExperiment() const {
    return this ->experimentComplete_.load();
}

void CNCOverseer::onCNCStatusChange(ProgramStatus oldStatus, ProgramStatus newStatus) {
    currentProgramStatus_ = newStatus;

    std::cout << "Overseer: CNC status change "
        << hurcoConnection_->programStatusToString(oldStatus) << " -> "
        << hurcoConnection_->programStatusToString(newStatus) << std::endl;

    if (newStatus == ProgramStatus::COMPLETED_SUCCESSFUL ||
        newStatus == ProgramStatus::COMPLETED_ERROR ||
        newStatus == ProgramStatus::COMPLETED_ABORT) {

        std::cout << "Overseer: CNC program completed - signaling experiment termination" << std::endl;
        experimentComplete_ = true;
    }
}