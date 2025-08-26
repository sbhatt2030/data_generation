#include "core/CNCExperimentRunner.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <filesystem>

namespace fs = std::filesystem;
// At top of CNCExperimentRunner.cpp
static size_t g_motionPointsInjected = 0;
static size_t g_motionPointsExtracted = 0;
static int g_logCounter = 0;

// =============================================================================
// SystemConfiguration Implementation
// =============================================================================



// =============================================================================
// ExperimentConfig Implementation
// =============================================================================

ExperimentConfig::ExperimentConfig() {
    // Set default G-code generation parameters
    experimentId = "ExpId";
    familyId = "FId";
    gcodeParams.num_trajectories = 10;
    gcodeParams.trajectory_type = TrajectoryType::MIXED;
    gcodeParams.circular_direction = CircularDirection::RANDOM;
    gcodeParams.arc_geometry = ArcGeometry::RANDOM;
    gcodeParams.max_trajectory_time = 2.0;
    gcodeParams.dwell_time = 0.5;
    gcodeParams.linear_probability = 0.6;
    gcodeParams.use_dwell_commands = true;
    gcodeParams.write_summary_to_file = true;

    // Set default noise parameters
    noiseParams.min_amplitude = 0.001;
    noiseParams.max_amplitude = 0.005;
    noiseParams.min_frequency = 0.5;
    noiseParams.max_frequency = 25.0;
    noiseParams.min_num_sines = 3;
    noiseParams.max_num_sines = 8;
    noiseParams.sparse_probability = 0.03;
    noiseParams.max_deviation_magnitude = 0.9;
    // Set default VFF configuration
    vffConfig.useVffGenerator = true;
    vffConfig.vffType = VffType::SMOOTH_GAUSSIAN;
    vffConfig.minAmplitude = 0.1;
    vffConfig.maxAmplitude = 10.0;
    vffConfig.minAlpha = 0.01;
    vffConfig.maxAlpha = 0.2;
    vffConfig.usePerAxisVff = false;
    vffConfig.fixedAmplitudes = { 0.0, 0.0, 0.0 };
    vffConfig.fixedAlphas = { 0.0, 0.0, 0.0 };
}

// =============================================================================
// CNCExperimentRunner Implementation
// =============================================================================

CNCExperimentRunner::CNCExperimentRunner()
    : configurationLoaded_(false)
    , mainLoopCounter_(0)
    , loggingInterval_(200) {

    std::cout << "CNC Experiment Runner initialized" << std::endl;
}

CNCExperimentRunner::~CNCExperimentRunner() {
    if (stopRequested_.load()) {
        std::cout << "Experiment runner shutting down..." << std::endl;
        performFinalCleanup();
    }
}

bool CNCExperimentRunner::loadSystemConfiguration(const std::string& configFile) {

    std::cout << "Using default system configuration with central constants" << std::endl;

    // Calculate logging interval based on frequencies
    loggingInterval_ = static_cast<int>(systemConfig_.systemTiming.mainLoopFrequency /
        systemConfig_.systemTiming.loggingFrequency);

    configurationLoaded_ = true;

    std::cout << "System configuration loaded successfully" << std::endl;
    std::cout << "  Main loop frequency: " << systemConfig_.systemTiming.mainLoopFrequency << " Hz" << std::endl;
    std::cout << "  Logging frequency: " << systemConfig_.systemTiming.loggingFrequency << " Hz" << std::endl;
    std::cout << "  Injection buffer capacity: " << systemConfig_.bufferConfig.injectionBufferCapacity << " points" << std::endl;
    configurationLoaded_ = true;

    return initializeSystemComponents();
}

bool CNCExperimentRunner::runExperiment(const ExperimentConfig& experimentConfig) {
    if (!configurationLoaded_) {
        std::cerr << "ERROR: System configuration not loaded. Call loadSystemConfiguration() first." << std::endl;
        return false;
    }

    // Reset experiment state
    stopRequested_.store(false);
    lastResult_ = ExperimentResult{};
    experimentStartTime_ = std::chrono::steady_clock::now();
    mainLoopCounter_ = 0;

    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STARTING CNC DATA COLLECTION EXPERIMENT" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    // Log experiment configuration
    std::cout << "Experiment Parameters:" << std::endl;
    std::cout << "  Output Directory: " << experimentConfig.outputDirectory << std::endl;
    std::cout << "  G-code Trajectories: " << experimentConfig.gcodeParams.num_trajectories << std::endl;
    std::cout << "  Noise Type: " << static_cast<int>(experimentConfig.noiseType) << std::endl;
    std::cout << "  VFF Generation: " << (experimentConfig.vffConfig.useVffGenerator ? "Enabled" : "Disabled") << std::endl;

    // Resolve and log seed configuration
    SeedConfiguration resolvedSeeds = resolveSeeds(experimentConfig.seedConfig);
    logSeedConfiguration(resolvedSeeds);

    try {
        // Phase 1: Initialize MotionService
        std::cout << "\nPhase 1: Initializing MotionService..." << std::endl;
        if (!initializeMotionService()) {
            setExperimentError("Failed to initialize MotionService");
            return false;
        }

        // Phase 2: Initialize all pipelines
        std::cout << "\nPhase 2: Initializing pipelines..." << std::endl;
        if (!initializePipelines(experimentConfig)) {
            setExperimentError("Failed to initialize pipelines");
            return false;
        }

        // Phase 3: Execute main processing loop
        std::cout << "\nPhase 3: Executing main processing loop..." << std::endl;
        if (!executeMainLoop()) {
            setExperimentError("Main loop execution failed");
            return false;
        }

        // Phase 4: Final cleanup and result compilation
        std::cout << "\nPhase 4: Final cleanup..." << std::endl;
        performFinalCleanup();

        // Compile final results
        updateExperimentResult();
        lastResult_.success = true;

        std::cout << "\n" << std::string(80, '=') << std::endl;
        std::cout << "EXPERIMENT COMPLETED SUCCESSFULLY" << std::endl;
        std::cout << "Session folder: " << lastResult_.sessionFolder << std::endl;
        std::cout << "Execution time: " << std::fixed << std::setprecision(1)
            << lastResult_.executionTimeSeconds << " seconds" << std::endl;
        std::cout << "CSV files written: " << lastResult_.csvFilesWritten << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        return true;

    }
    catch (const std::exception& e) {
        setExperimentError("Exception during experiment: " + std::string(e.what()));
        performFinalCleanup();
        return false;
    }
}

void CNCExperimentRunner::requestStop() {
    stopRequested_.store(true);
    std::cout << "Graceful shutdown requested..." << std::endl;
}

// =============================================================================
// Private Implementation Methods
// =============================================================================

bool CNCExperimentRunner::initializeMotionService() {
    try {
        motionService_ = std::make_unique<MotionService>();

        int errorCode = 0;
        MOT_SERVICE_RETURN_CODE result = motionService_->InitMotionService(&errorCode);

        if (result != MOT_SERVICE_INIT_SUCCESS) {
            std::cerr << "ERROR: MotionService initialization failed with error code: " << errorCode << std::endl;
            return false;
        }

        std::cout << "MotionService initialized successfully" << std::endl;
        return true;

    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Exception initializing MotionService: " << e.what() << std::endl;
        return false;
    }
}

bool CNCExperimentRunner::initializePipelines(const ExperimentConfig& experimentConfig) {
    try {
        // Initialize Generation Pipeline (existing logic)
        std::cout << "Initializing Generation Pipeline..." << std::endl;
        generationPipeline_ = std::make_unique<GenerationPipeline>(
            experimentConfig.outputDirectory,
            resolvedGcodeGeneratorSeed_,
            resolvedNoiseGeneratorSeed_,
            resolvedVffGeneratorSeed_);

        // Configure generation pipeline (existing)
        generationPipeline_->setMachineConstraints(systemConfig_.machineConstraints);
        generationPipeline_->setMotionConfig(systemConfig_.motionConfig);
        generationPipeline_->setNoiseParams(experimentConfig.noiseParams);
        generationPipeline_->setNoiseType(experimentConfig.noiseType);
        generationPipeline_->setVffConfig(experimentConfig.vffConfig);

        if (!generationPipeline_->initialize(experimentConfig.existingGcodeFile, experimentConfig.gcodeParams)) {
            std::cerr << "ERROR: Failed to initialize Generation Pipeline" << std::endl;
            return false;
        }


        // Initialize Injection Pipeline (existing)
        std::cout << "Initializing Injection Pipeline..." << std::endl;
        injectionPipeline_ = std::make_unique<InjectionPipeline>();

        if (!injectionPipeline_->initialize(motionService_.get(), generationPipeline_.get())) {
            std::cerr << "ERROR: Failed to initialize Injection Pipeline" << std::endl;
            return false;
        }

        // Initialize Extraction Pipeline (existing)
        std::cout << "Initializing Extraction Pipeline..." << std::endl;
        extractionPipeline_ = std::make_unique<ExtractionPipeline>();

        std::set<int> expectedLineNumbers = generationPipeline_->getAllLineNumbers();

        std::string logsDir = getLogsDirectory();
        std::string spikeLogPath = logsDir + "\\position_spikes.log";

        if (!extractionPipeline_->initialize(motionService_.get(),
            generationPipeline_->getSessionFolder(),
            expectedLineNumbers,
            spikeLogPath)) {
            std::cerr << "ERROR: Failed to initialize Extraction Pipeline" << std::endl;
            return false;
        }

        initializeLogging();

        std::cout << "All pipelines initialized successfully" << std::endl;
        std::cout << "  Session folder: " << generationPipeline_->getSessionFolder() << std::endl;
        std::cout << "  Expected line numbers: " << expectedLineNumbers.size() << std::endl;
        std::cout << "  Continuous generation: " << (generationPipeline_->isContinuousMode() ? "ENABLED" : "DISABLED") << std::endl;
        return true;

    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Exception initializing pipelines: " << e.what() << std::endl;
        return false;
    }
}

SeedConfiguration CNCExperimentRunner::resolveSeeds(const SeedConfiguration& seedConfig) {
    SeedConfiguration resolved = seedConfig;

    // Set up master RNG if master seed is specified
    std::mt19937 masterRng;
    if (seedConfig.masterSeed.has_value()) {
        masterRng.seed(*seedConfig.masterSeed);
    }
    else {
        std::random_device rd;
        masterRng.seed(rd());
    }

    // Resolve each seed using priority: individual > master-derived > auto-generated
    auto resolveSeed = [&](std::optional<unsigned int> individualSeed) -> unsigned int {
        if (individualSeed.has_value()) {
            return *individualSeed;  // Use specified individual seed
        }
        if (seedConfig.masterSeed.has_value()) {
            return masterRng();      // Derive from master seed
        }
        return std::random_device{}();  // Auto-generate
        };

    resolvedGcodeGeneratorSeed_ = resolveSeed(seedConfig.gcodeGeneratorSeed);
    resolvedNoiseGeneratorSeed_ = resolveSeed(seedConfig.noiseGeneratorSeed);
    resolvedVffGeneratorSeed_ = resolveSeed(seedConfig.vffGeneratorSeed);

    return resolved;
}

bool CNCExperimentRunner::executeMainLoop() {
    std::cout << "Starting main processing loop at "
        << systemConfig_.systemTiming.mainLoopFrequency << " Hz..." << std::endl;

    const auto loopPeriod = std::chrono::microseconds(
        static_cast<long>(1000000.0 / systemConfig_.systemTiming.mainLoopFrequency));

    auto nextLoopTime = std::chrono::steady_clock::now() + loopPeriod;

    while (shouldContinueExperiment()) {
        auto loopStart = std::chrono::steady_clock::now();

        // Perform single processing cycle
        performSingleCycle();

        // Periodic logging
        if (mainLoopCounter_ % loggingInterval_ == 0) {
            logAggregatedSystemStatus();
        }

        mainLoopCounter_++;

        // Maintain loop timing
        std::this_thread::sleep_until(nextLoopTime);
        nextLoopTime += loopPeriod;

        // Check for timing issues
        auto loopEnd = std::chrono::steady_clock::now();
        auto loopDuration = std::chrono::duration_cast<std::chrono::microseconds>(loopEnd - loopStart);
        //if (loopDuration > loopPeriod * 1.1) {  // 10% tolerance
        //    std::cout << "WARNING: Loop cycle took " << loopDuration.count()
        //        << " us (target: " << loopPeriod.count() << " us)" << std::endl;
        //}

        // In CNCExperimentRunner::performSingleCycle()
        
    }

    std::cout << "Main processing loop completed after " << mainLoopCounter_ << " cycles" << std::endl;
    return true;
}

void CNCExperimentRunner::performSingleCycle() {
    // Process pipelines in optimal order for data flow
    extractionPipeline_->processOneCycle();   // Empty SMR output buffer first
    injectionPipeline_->processOneCycle();    // Fill SMR input buffer
}

bool CNCExperimentRunner::shouldContinueExperiment() {
    // 1. Manual stop request
    if (stopRequested_.load()) {
        std::cout << "Manual stop requested" << std::endl;
        return false;
    }

    // 2. GRACE PERIOD: Don't check CNC completion for first 5 seconds
    auto elapsedTime = getElapsedTimeSeconds();
    if (elapsedTime < 10.0) {
        return true;  // Skip CNC checks during startup
    }

    if (cncConnection_ && cncConnection_->isProgramComplete()) {
        std::cout << "CNC completed - signaling extraction to stop" << std::endl;
        if (extractionPipeline_) {
            extractionPipeline_->setComplete();  // Set the completion flag
        }
        return false;
    }

    return true;
}

void CNCExperimentRunner::initializeLogging() {
    std::string logsDir = getLogsDirectory();
    fs::create_directories(logsDir);

    // Create log files in logs/ directory
    experimentLogFile_ = logsDir + "/experiment_log.txt";
    systemStatusFile_ = logsDir + "/system_status.txt";
}

void CNCExperimentRunner::logAggregatedSystemStatus() {
    auto elapsedTime = getElapsedTimeSeconds();

    // Get pipeline statistics
    auto injectionStats = injectionPipeline_->getStats();
    auto extractionStats = extractionPipeline_->getStats();

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "SYSTEM STATUS (Elapsed: " << std::fixed << std::setprecision(1)
        << elapsedTime << "s)" << std::endl;
    std::cout << std::string(60, '=') << std::endl;


    // Generation status
    std::cout << "Generation: ";
    if (generationPipeline_->hasMoreCommands()) {
        if (generationPipeline_->isContinuousMode()) {
            std::cout << "Continuous (infinite)" << std::endl;
        }
        else {
            std::cout << "Active (" << generationPipeline_->getRemainingCommands() << " commands remaining)" << std::endl;
        }
    }
    else {
        std::cout << "Complete (" << injectionStats.trajectoriesGenerated << " trajectories)" << std::endl;
    }

    // Injection status
    std::cout << "Injection:  Buffer: " << injectionStats.currentBufferSize
        << "/" << systemConfig_.bufferConfig.injectionBufferCapacity
        << " (" << std::fixed << std::setprecision(0) << (injectionStats.bufferFillLevel * 100) << "%) | "
        << "Written: " << injectionStats.totalPointsWritten << " points" << std::endl;

    // Extraction status
    std::cout << "Extraction: Buffer: " << extractionStats.currentBufferSize
        << "/" << ExtractionConfig::FIXED_BUFFER_SIZE
        << " (" << std::fixed << std::setprecision(0)
        << (100.0 * extractionStats.currentBufferSize / ExtractionConfig::FIXED_BUFFER_SIZE) << "%)";

    if (extractionStats.currentLineNumber >= 0) {
        std::cout << " | Line: " << extractionStats.currentLineNumber;
    }
    std::cout << " | Files: " << extractionStats.csvFilesWritten << std::endl;

    // System health
    std::cout << "System Health: ";
    bool healthy = true;

    if (injectionStats.writeFailures > 0 && injectionStats.writeAttempts > 0) {
        double failureRate = static_cast<double>(injectionStats.writeFailures) / injectionStats.writeAttempts;
        if (failureRate > 0.3) {
            std::cout << "Injection Issues ";
            healthy = false;
        }
    }

    if (extractionStats.readFailures > 0 && extractionStats.readAttempts > 0) {
        double failureRate = static_cast<double>(extractionStats.readFailures) / extractionStats.readAttempts;
        if (failureRate > 0.3) {
            std::cout << "Extraction Issues ";
            healthy = false;
        }
    }

    if (healthy) {
        std::cout << "Healthy";
    }

    std::cout << std::endl;
    std::cout << "Motion Points: Injected=" << injectionStats.totalPointsWritten
        << ", Extracted=" << extractionStats.pointsWithMotion << std::endl;

    std::cout << std::string(60, '=') << std::endl;

    // Log to file (existing logic)
    try {
        std::ofstream logFile(systemStatusFile_, std::ios::app);
        if (logFile.is_open()) {
            logFile << getCurrentTimestamp() << " - Elapsed: " << elapsedTime << "s" << std::endl;
            logFile << "Generation: " << (generationPipeline_->hasMoreCommands() ? "Active" : "Complete") << std::endl;
            logFile << "Injection Buffer: " << injectionStats.currentBufferSize << "/"
                << systemConfig_.bufferConfig.injectionBufferCapacity << std::endl;
            logFile << "Extraction Files: " << extractionStats.csvFilesWritten << std::endl;
            logFile << "---" << std::endl;
            logFile.close();
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Warning: Could not write to log file: " << e.what() << std::endl;
    }
}

// MODIFIED: Enhanced final cleanup with external control awareness
void CNCExperimentRunner::performFinalCleanup() {
    std::cout << "Performing final cleanup..." << std::endl;
    // Standard cleanup for autonomous mode
    if (injectionPipeline_) {
        injectionPipeline_->finalFlush();
    }
   
    if (extractionPipeline_) {
        extractionPipeline_->finalFlush();
    }
    // Update final results
    updateExperimentResult();

    std::cout << "Final cleanup complete" << std::endl;
    std::cout << "  Session folder: " << lastResult_.sessionFolder << std::endl;
}


void CNCExperimentRunner::updateExperimentResult() {
    lastResult_.executionTimeSeconds = getElapsedTimeSeconds();

    if (generationPipeline_) {
        lastResult_.sessionFolder = generationPipeline_->getSessionFolder();
    }

    // Get final statistics from pipelines
    if (injectionPipeline_) {
        lastResult_.finalInjectionStats = injectionPipeline_->getStats();
    }

    if (extractionPipeline_) {
        lastResult_.finalExtractionStats = extractionPipeline_->getStats();
    }

    // Aggregate metrics
    lastResult_.totalTrajectoriesGenerated = lastResult_.finalInjectionStats.trajectoriesGenerated;
    lastResult_.totalPointsCollected = lastResult_.finalExtractionStats.totalPointsKept;
    lastResult_.csvFilesWritten = lastResult_.finalExtractionStats.csvFilesWritten;

    // Record actual seeds used
    lastResult_.actualGcodeGeneratorSeed = resolvedGcodeGeneratorSeed_;
    lastResult_.actualNoiseGeneratorSeed = resolvedNoiseGeneratorSeed_;
    lastResult_.actualVffGeneratorSeed = resolvedVffGeneratorSeed_;

    // Calculate aggregate health metrics
    if (lastResult_.finalInjectionStats.writeAttempts > 0) {
        lastResult_.avgWriteSuccessRate = 100.0 *
            (lastResult_.finalInjectionStats.writeAttempts - lastResult_.finalInjectionStats.writeFailures) /
            lastResult_.finalInjectionStats.writeAttempts;
        lastResult_.totalWriteFailures = lastResult_.finalInjectionStats.writeFailures;
    }

    if (lastResult_.finalExtractionStats.readAttempts > 0) {
        lastResult_.avgReadSuccessRate = 100.0 *
            (lastResult_.finalExtractionStats.readAttempts - lastResult_.finalExtractionStats.readFailures) /
            lastResult_.finalExtractionStats.readAttempts;
        lastResult_.totalReadFailures = lastResult_.finalExtractionStats.readFailures;
    }
}

void CNCExperimentRunner::logSeedConfiguration(const SeedConfiguration& seedConfig) {
    std::cout << "\n=== Seed Configuration ===" << std::endl;

    if (seedConfig.masterSeed.has_value()) {
        std::cout << "Master Seed: " << *seedConfig.masterSeed << " (specified)" << std::endl;
    }
    else {
        std::cout << "Master Seed: None (auto-generated individual seeds)" << std::endl;
    }

    auto logSeed = [&](const std::string& name, std::optional<unsigned int> individualSeed, unsigned int resolvedSeed) {
        std::cout << name << ": " << resolvedSeed;
        if (individualSeed.has_value()) {
            std::cout << " (individually specified)";
        }
        else if (seedConfig.masterSeed.has_value()) {
            std::cout << " (derived from master)";
        }
        else {
            std::cout << " (auto-generated)";
        }
        std::cout << std::endl;
        };

    logSeed("G-code Generator Seed", seedConfig.gcodeGeneratorSeed, resolvedGcodeGeneratorSeed_);
    logSeed("Noise Generator Seed", seedConfig.noiseGeneratorSeed, resolvedNoiseGeneratorSeed_);
    logSeed("VFF Generator Seed", seedConfig.vffGeneratorSeed, resolvedVffGeneratorSeed_);

    // Determine resolution strategy
    std::string strategy;
    if (seedConfig.usesMasterSeed() && !seedConfig.usesIndividualSeeds()) {
        strategy = "Master seed only";
    }
    else if (!seedConfig.usesMasterSeed() && seedConfig.usesIndividualSeeds()) {
        strategy = "Individual seeds only";
    }
    else if (seedConfig.usesMasterSeed() && seedConfig.usesIndividualSeeds()) {
        strategy = "Mixed (master + individual overrides)";
    }
    else {
        strategy = "Auto-generated";
    }

    std::cout << "Seed Resolution: " << strategy << std::endl;
    std::cout << "===========================" << std::endl;
}

std::string CNCExperimentRunner::getLogsDirectory() const {
    if (generationPipeline_) {
        return generationPipeline_->getSessionFolder() + "\\logs";
    }
    return ".\\logs"; // fallback
}

void CNCExperimentRunner::shutdownPipelines() {
    std::cout << "Shutting down pipelines..." << std::endl;

    // Shutdown in reverse order of initialization
    extractionPipeline_.reset();
    injectionPipeline_.reset();
    generationPipeline_.reset();

    std::cout << "Pipelines shutdown complete" << std::endl;
}

void CNCExperimentRunner::shutdownMotionService() {
    std::cout << "Shutting down MotionService..." << std::endl;

    if (motionService_) {
        // MotionService destructor handles cleanup
        motionService_.reset();
    }

    std::cout << "MotionService shutdown complete" << std::endl;
}

bool CNCExperimentRunner::handleComponentError(const std::string& component, const std::string& error) {
    std::cerr << "ERROR in " << component << ": " << error << std::endl;

    // For now, all component errors are fatal
    // In future versions, could implement recovery strategies
    setExperimentError("Component failure in " + component + ": " + error);
    return false;
}

void CNCExperimentRunner::setExperimentError(const std::string& errorMessage) {
    lastResult_.success = false;
    lastResult_.errorMessage = errorMessage;
    lastResult_.executionTimeSeconds = getElapsedTimeSeconds();

    std::cerr << "EXPERIMENT FAILED: " << errorMessage << std::endl;
}

std::string CNCExperimentRunner::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    std::tm local_time;
    localtime_s(&local_time, &time_t);
    ss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

double CNCExperimentRunner::getElapsedTimeSeconds() const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - experimentStartTime_);
    return elapsed.count() / 1000.0;
}


std::string CNCExperimentRunner::getGeneratedGCodePath() const {
    if (generationPipeline_) {
        return generationPipeline_->getGCodeFilePath();
    }
    return "";  // Return empty string if pipeline not initialized
}

bool CNCExperimentRunner::initializeExperiment(const ExperimentConfig& experimentConfig) {
    if (!configurationLoaded_) {
        std::cerr << "ERROR: System configuration not loaded. Call loadSystemConfiguration() first." << std::endl;
        return false;
    }

    // CHANGE: Ensure persistent system components are initialized (once per batch)
    if (!initializeSystemComponents()) {
        return false;
    }

    // Reset experiment state
    stopRequested_.store(false);
    lastResult_ = ExperimentResult{};
    experimentStartTime_ = std::chrono::steady_clock::now();
    mainLoopCounter_ = 0;

    std::cout << "Initializing CNC experiment (G-code generation phase)..." << std::endl;

    // Log experiment configuration
    std::cout << "Experiment Parameters:" << std::endl;
    std::cout << "  Output Directory: " << experimentConfig.outputDirectory << std::endl;
    std::cout << "  G-code Trajectories: " << experimentConfig.gcodeParams.num_trajectories << std::endl;
    std::cout << "  Noise Type: " << static_cast<int>(experimentConfig.noiseType) << std::endl;
    std::cout << "  VFF Generation: " << (experimentConfig.vffConfig.useVffGenerator ? "Enabled" : "Disabled") << std::endl;

    // Resolve and log seed configuration
    SeedConfiguration resolvedSeeds = resolveSeeds(experimentConfig.seedConfig);
    logSeedConfiguration(resolvedSeeds);

    try {
        // REMOVE: Phase 1 - MotionService already initialized in initializeSystemComponents()

        // CHANGE: Phase 2 - Only create fresh GenerationPipeline and reconfigure existing pipelines
        std::cout << "Phase 2: Creating fresh GenerationPipeline and reconfiguring pipelines..." << std::endl;
        if (!createFreshGenerationPipelineAndReconfigure(experimentConfig)) {
            setExperimentError("Failed to create/reconfigure pipelines");
            return false;
        }

        InjectionPipeline::resetLineCounter();

        std::cout << "Experiment initialization complete!" << std::endl;
        std::cout << "  G-code file: " << generationPipeline_->getGCodeFilePath() << std::endl;
        std::cout << "  Session folder: " << generationPipeline_->getSessionFolder() << std::endl;

        return true;

    }
    catch (const std::exception& e) {
        setExperimentError("Exception during experiment initialization: " + std::string(e.what()));
        return false;
    }
}

bool CNCExperimentRunner::startExperimentLoop() {
    if (!generationPipeline_ || !injectionPipeline_ || !extractionPipeline_ || !motionService_) {
        std::cerr << "ERROR: Experiment not initialized. Call initializeExperiment() first." << std::endl;
        return false;
    }

    std::cout << "Starting data collection main loop..." << std::endl;

    try {
        // This will block until external termination callback returns true
        if (!executeMainLoop()) {
            setExperimentError("Main loop execution failed");
            return false;
        }

        // Cleanup and finalize
        performFinalCleanup();
        updateExperimentResult();
        lastResult_.success = true;

        std::cout << "✅ Data collection completed!" << std::endl;
        std::cout << "  Duration: " << std::fixed << std::setprecision(1)
            << lastResult_.executionTimeSeconds << "s" << std::endl;
        std::cout << "  Files written: " << lastResult_.csvFilesWritten << std::endl;

        return true;

    }
    catch (const std::exception& e) {
        setExperimentError("Exception in main loop: " + std::string(e.what()));
        performFinalCleanup();
        return false;
    }
}

bool CNCExperimentRunner::initializeSystemComponents() {
    if (systemComponentsInitialized_) {
        return true; // Already initialized
    }

    std::cout << "Initializing persistent system components..." << std::endl;

    // Initialize MotionService (expensive SMR connection)
    if (!initializeMotionService()) {
        setExperimentError("Failed to initialize MotionService");
        return false;
    }

    // Create persistent pipelines (empty, will be configured per experiment)
    injectionPipeline_ = std::make_unique<InjectionPipeline>();
    extractionPipeline_ = std::make_unique<ExtractionPipeline>();

    systemComponentsInitialized_ = true;
    std::cout << "✅ Persistent system components initialized" << std::endl;
    return true;
}

bool CNCExperimentRunner::createFreshGenerationPipelineAndReconfigure(const ExperimentConfig& experimentConfig) {
    std::cout << "Creating fresh GenerationPipeline and reconfiguring persistent pipelines..." << std::endl;

    // Resolve seeds for this experiment
    SeedConfiguration resolvedSeeds = resolveSeeds(experimentConfig.seedConfig);

    // Create fresh GenerationPipeline for this experiment
    generationPipeline_ = std::make_unique<GenerationPipeline>(
        experimentConfig.outputDirectory,
        resolvedSeeds.gcodeGeneratorSeed,
        resolvedSeeds.noiseGeneratorSeed,
        resolvedSeeds.vffGeneratorSeed);

    // Configure the generation pipeline
    generationPipeline_->setMachineConstraints(systemConfig_.machineConstraints);
    generationPipeline_->setMotionConfig(systemConfig_.motionConfig);
    generationPipeline_->setNoiseParams(experimentConfig.noiseParams);
    generationPipeline_->setNoiseType(experimentConfig.noiseType);
    generationPipeline_->setVffConfig(experimentConfig.vffConfig);

    // Initialize the generation pipeline
    if (!generationPipeline_->initialize(experimentConfig.existingGcodeFile, experimentConfig.gcodeParams)) {
        setExperimentError("Failed to initialize GenerationPipeline");
        return false;
    }

    // Now reconfigure the persistent pipelines to point to this new generation pipeline
    return reconfigurePersistentPipelines(experimentConfig);
}

bool CNCExperimentRunner::reconfigurePersistentPipelines(const ExperimentConfig& experimentConfig) {
    std::cout << "Reconfiguring persistent pipelines for new experiment..." << std::endl;

    // Reconfigure injection pipeline to point to new generation pipeline
    if (!injectionPipeline_->reconfigure(motionService_.get(), generationPipeline_.get())) {
        setExperimentError("Failed to reconfigure Injection Pipeline");
        return false;
    }

    // Reconfigure extraction pipeline with new session folder and expected line numbers
    std::set<int> expectedLineNumbers = generationPipeline_->getAllLineNumbers();
    std::string logsDir = getLogsDirectory();
    std::string spikeLogPath = logsDir + "\\position_spikes.log";

    if (!extractionPipeline_->reconfigure(
        motionService_.get(),
        generationPipeline_->getSessionFolder(),
        expectedLineNumbers,
        spikeLogPath)) {
        setExperimentError("Failed to reconfigure Extraction Pipeline");
        return false;
    }

    std::cout << "Persistent pipelines reconfigured successfully" << std::endl;
    return true;
}
