#include "core/GenerationPipeline.hpp"
#include "core/VffGenerator.hpp"
#include "core/input_data_point.hpp"
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <random>
#include <iostream>

namespace fs = std::filesystem;

GenerationPipeline::GenerationPipeline(const std::string& outputFolder,
            unsigned int gcodeGeneratorSeed,
            unsigned int noiseGeneratorSeed,
            unsigned int vffGeneratorSeed): 
    baseOutputFolder_(outputFolder), 
    noiseType_(KinematicNoiseType::SMOOTH_GAUSSIAN_BANDPASS),
    gcodeGeneratorSeed_(gcodeGeneratorSeed),
    noiseGeneratorSeed_(noiseGeneratorSeed),
    vffGeneratorSeed_(vffGeneratorSeed) {

    // NO MORE master seed derivation - use the provided resolved seeds directly
    std::cout << "GenerationPipeline initialized with resolved seeds:" << std::endl;
    std::cout << "  G-code Generator: " << gcodeGeneratorSeed_ << std::endl;
    std::cout << "  Noise Generator: " << noiseGeneratorSeed_ << std::endl;
    std::cout << "  VFF Generator: " << vffGeneratorSeed_ << std::endl;
}

bool GenerationPipeline::initialize(const std::string& gcodeFile, const GenerationParams& genParams) {
    std::cout << "Initializing Simplified Generation Pipeline (Continuous Noise Mode)..." << std::endl;

    // Step 1: Create unique session directory
    if (!createUniqueSessionFolder()) {
        std::cerr << "ERROR: Failed to create unique session folder" << std::endl;
        return false;
    }

    // Step 2: Handle G-code (copy existing or generate new) - for session organization only
    if (!gcodeFile.empty()) {
        if (!copyGCodeFile(gcodeFile)) {
            std::cerr << "ERROR: Failed to copy G-code file: " << gcodeFile << std::endl;
            return false;
        }
        std::cout << "Copied G-code file for session reference: " << gcodeFile << std::endl;
    }
    else {
        if (!generateGCodeFile(genParams)) {
            std::cerr << "ERROR: Failed to generate G-code file" << std::endl;
            return false;
        }
        std::cout << "Generated G-code file for session reference" << std::endl;
    }

    // Step 3: Save configuration and seed information
    if (!saveConfigurationFiles(genParams, !gcodeFile.empty())) {
        std::cerr << "ERROR: Failed to save configuration files" << std::endl;
        return false;
    }

    // Step 4: Initialize noise generators
    initializeGenerators();

    if (noiseGenerator_) {
        noiseGenerator_->resetContinuousState();
    }
    if (vffGenerator_) {
        vffGenerator_->resetContinuousState();
    }

    // Step 5: Enable continuous mode
    enableContinuousGeneration(true);

    std::cout << "Simplified Generation Pipeline initialization complete:" << std::endl;
    std::cout << "  Session folder: " << uniqueSessionFolder_ << std::endl;
    std::cout << "  G-code file: " << gcodeFilePath_ << std::endl;
    std::cout << "  Mode: Continuous noise generation" << std::endl;
    std::cout << "  Master seed: " << masterSeed_ << std::endl;
    std::cout << "  VFF generation: " << (vffConfig_.useVffGenerator ? "Enabled" : "Disabled") << std::endl;

    return true;
}

void GenerationPipeline::enableContinuousGeneration(bool enable) {
    if (enable) {
        std::cout << "GenerationPipeline: Continuous noise generation ENABLED" << std::endl;
        std::cout << "  - Will generate noise chunks independently" << std::endl;
        std::cout << "  - Noise type: " << static_cast<int>(noiseType_) << std::endl;
        std::cout << "  - Max Amplitude: " << noiseParams_.max_amplitude << " mm" << std::endl;

        // Reset continuous state
        continuousChunkCounter_ = 0;

        // Reset noise generator state for clean continuous generation
        if (noiseGenerator_) {
            noiseGenerator_->resetContinuousState();
        }
    }
}

std::set<int> GenerationPipeline::getAllLineNumbers() const {
    std::set<int> lineNumbers;
    lineNumbers.insert(1);  // Return dummy line number for extraction pipeline
    return lineNumbers;
}

std::vector<InputDataPoint> GenerationPipeline::generateNextCommandData() {
    return generateContinuousNoiseChunk();
}

std::vector<InputDataPoint> GenerationPipeline::generateContinuousNoiseChunk() {
    try {
        std::cout << "Generating continuous noise chunk " << continuousChunkCounter_
            << " (type: " << static_cast<int>(noiseType_) << ")" << std::endl;

        // Generate noise chunk using enhanced noise generator
        std::vector<InputDataPoint> noiseChunk = noiseGenerator_->generateNoiseChunk(
            TrajectoryKinematics::FIXED_LENGTH,  // 8000 points
            noiseParams_,
            motionConfig_.getTimeStep(),         // 0.00025s (4kHz)
            continuousChunkCounter_              // Line number for organization
        );

        // Add VFF data if enabled
        if (vffConfig_.useVffGenerator && vffGenerator_) {
            addVFFToChunk(noiseChunk);
        }

        continuousChunkCounter_++;

        std::cout << "Generated continuous noise chunk: " << noiseChunk.size()
            << " points, line " << (continuousChunkCounter_ - 1) << std::endl;

        return noiseChunk;

    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Exception generating continuous noise chunk: " << e.what() << std::endl;
        return {}; // Return empty vector on error
    }
}

void GenerationPipeline::addVFFToChunk(std::vector<InputDataPoint>& chunk) {
    if (chunk.empty() || !vffGenerator_) {
        return;
    }

    // ADD THESE CHECKS:
    if (!vffConfig_.useVffGenerator || vffConfig_.vffType == VffType::NO_VFF) {
        return;  // Skip VFF generation
    }

    try {
        // Generate VFF signals for the chunk
        std::array<std::vector<double>, 3> vffSignals;

        if (vffConfig_.usePerAxisVff) {
            vffSignals = vffGenerator_->generateAllAxes(
                vffConfig_.fixedAmplitudes,
                vffConfig_.fixedAlphas);
        }
        else {
            vffSignals = vffGenerator_->generateAllAxes();
        }

        // Add VFF data to each point in the chunk
        size_t chunkSize = chunk.size();
        for (size_t i = 0; i < chunkSize && i < vffSignals[0].size(); ++i) {
            chunk[i].vff_x = vffSignals[0][i];
            chunk[i].vff_y = vffSignals[1][i];
            chunk[i].vff_z = vffSignals[2][i];
        }

        std::cout << "Added VFF data to continuous noise chunk" << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "WARNING: Failed to add VFF data to chunk: " << e.what() << std::endl;
        // Continue without VFF data
    }
}

// Configuration setters
void GenerationPipeline::setVffConfig(const VffConfig& config) {
    vffConfig_ = config;
    if (vffConfig_.useVffGenerator && vffGenerator_) {
        vffGenerator_ = std::make_unique<VffGenerator>(vffGeneratorSeed_);
    }
}

void GenerationPipeline::setNoiseParams(const KinematicNoiseGenerator::NoiseParams& params) {
    noiseParams_ = params;
    if (noiseGenerator_) {
        noiseGenerator_ = std::make_unique<KinematicNoiseGenerator>(
            machineConstraints_, motionConfig_, noiseGeneratorSeed_);
    }
}

void GenerationPipeline::setMachineConstraints(const MachineConstraints& constraints) {
    machineConstraints_ = constraints;
    if (noiseGenerator_) {
        noiseGenerator_ = std::make_unique<KinematicNoiseGenerator>(
            constraints, motionConfig_, noiseGeneratorSeed_);
    }
}

void GenerationPipeline::setMotionConfig(const MotionConfig& config) {
    motionConfig_ = config;
    if (noiseGenerator_) {
        noiseGenerator_ = std::make_unique<KinematicNoiseGenerator>(
            machineConstraints_, config, noiseGeneratorSeed_);
    }
}

void GenerationPipeline::setNoiseType(KinematicNoiseType type) {
    noiseType_ = type;
    if (noiseGenerator_) {
        noiseGenerator_->setNoiseType(type);  // ADD THIS LINE
    }
}


void GenerationPipeline::initializeGenerators() {
    std::cout << "Initializing generators with resolved seeds:" << std::endl;

    noiseGenerator_ = std::make_unique<KinematicNoiseGenerator>(
        machineConstraints_, motionConfig_, noiseGeneratorSeed_);
    std::cout << "  Noise Generator initialized with seed: " << noiseGeneratorSeed_ << std::endl;

    noiseGenerator_->setNoiseType(noiseType_);

    if (vffConfig_.useVffGenerator) {
        vffGenerator_ = std::make_unique<VffGenerator>(vffGeneratorSeed_);
        std::cout << "VFF Generator initialized with seed: " << vffGeneratorSeed_ << std::endl;
    }

    std::cout << "Noise generators initialized successfully" << std::endl;
}

bool GenerationPipeline::createUniqueSessionFolder() {
    try {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        std::tm local_time;
        localtime_s(&local_time, &time_t);
        ss << "exp_" << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S");
        ss << "_seed" << masterSeed_;

        uniqueSessionFolder_ = (fs::path(baseOutputFolder_) / ss.str()).make_preferred().string();

        // CREATE ALL SUBDIRECTORIES
        fs::create_directories(uniqueSessionFolder_);
        fs::create_directories(uniqueSessionFolder_ + "\\config");
        fs::create_directories(uniqueSessionFolder_ + "\\gcode");
        fs::create_directories(uniqueSessionFolder_ + "\\results");
        fs::create_directories(uniqueSessionFolder_ + "\\logs");

        return fs::exists(uniqueSessionFolder_);
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Creating session folder: " << e.what() << std::endl;
        return false;
    }
}

bool GenerationPipeline::copyGCodeFile(const std::string& sourceFile) {
    if (!fs::exists(sourceFile)) {
        std::cerr << "ERROR: Source G-code file does not exist: " << sourceFile << std::endl;
        return false;
    }

    try {
        fs::path sourcePath(sourceFile);
        gcodeFilePath_ = (fs::path(uniqueSessionFolder_) / "gcode" / sourcePath.filename()).make_preferred().string();
        fs::copy_file(sourceFile, gcodeFilePath_);
        return fs::exists(gcodeFilePath_);
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Copying G-code file: " << e.what() << std::endl;
        return false;
    }
}

bool GenerationPipeline::generateGCodeFile(const GenerationParams& genParams) {
    try {
        GCodeGenerator generator(machineConstraints_, gcodeGeneratorSeed_);
        gcodeFilePath_ = (fs::path(uniqueSessionFolder_) / "gcode" / "generated.fnc").make_preferred().string();

        GenerationParams params = genParams;
        if (params.num_trajectories == 0) {
            params.num_trajectories = 10;
        }

        params.summary_output_directory = (fs::path(uniqueSessionFolder_) / "gcode").make_preferred().string();

        return generator.generateGCodeFile(gcodeFilePath_, params);
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Generating G-code file: " << e.what() << std::endl;
        return false;
    }
}

bool GenerationPipeline::saveConfigurationFiles(const GenerationParams& genParams, bool usedExistingGCode) {
    try {
        std::string configDir = (fs::path(uniqueSessionFolder_) / "config").make_preferred().string();

        // Save master seed and derived seeds
        std::ofstream seedFile(fs::path(configDir) / "seeds.txt");
        seedFile << "Master Seed: " << masterSeed_ << std::endl;
        seedFile << "G-code Generator Seed: " << gcodeGeneratorSeed_ << std::endl;
        seedFile << "Noise Generator Seed: " << noiseGeneratorSeed_ << std::endl;
        seedFile << "VFF Generator Seed: " << vffGeneratorSeed_ << std::endl;
        seedFile << "Timestamp: " << getCurrentTimestamp() << std::endl;
        seedFile.close();

        // Save other config files
        saveVffConfigToFile();
        saveNoiseConfigToFile();
        saveMachineConfigToFile();

        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Saving configuration files: " << e.what() << std::endl;
        return false;
    }
}

void GenerationPipeline::saveVffConfigToFile() const {
    try {
        std::string vffConfigPath = (fs::path(uniqueSessionFolder_) / "config" / "vff_config.txt").make_preferred().string();
        std::ofstream vffFile(vffConfigPath);

        if (!vffFile.is_open()) {
            std::cerr << "Warning: Could not save VFF configuration file" << std::endl;
            return;
        }

        vffFile << "=== VFF Generation Configuration ===" << std::endl;
        vffFile << "Use VFF Generator: " << (vffConfig_.useVffGenerator ? "Yes" : "No") << std::endl;
        vffFile << "VFF Type: " << static_cast<int>(vffConfig_.vffType) << std::endl;
        vffFile << "Min Amplitude: " << vffConfig_.minAmplitude << std::endl;
        vffFile << "Max Amplitude: " << vffConfig_.maxAmplitude << std::endl;

        vffFile.close();
        std::cout << "VFF configuration saved to: " << vffConfigPath << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Saving VFF configuration: " << e.what() << std::endl;
    }
}

void GenerationPipeline::saveNoiseConfigToFile() const {
    try {
        std::string configPath = (fs::path(uniqueSessionFolder_) / "config" / "noise_config.txt").make_preferred()
.string();
        std::ofstream configFile(configPath);

        if (!configFile.is_open()) {
            std::cerr << "Warning: Could not save noise configuration file" << std::endl;
            return;
        }

        configFile << "=== Noise Generation Configuration ===" << std::endl;
        configFile << "Noise Type: " << static_cast<int>(noiseType_) << std::endl;
        configFile << "Min Amplitude: " << noiseParams_.min_amplitude << " mm" << std::endl;
        configFile << "Max Amplitude: " << noiseParams_.max_amplitude << " mm" << std::endl;
        configFile << "Min Frequency: " << noiseParams_.min_frequency << " Hz" << std::endl;
        configFile << "Max Frequency: " << noiseParams_.max_frequency << " Hz" << std::endl;
        configFile << "Min Sines: " << noiseParams_.min_num_sines << std::endl;
        configFile << "Max Sines: " << noiseParams_.max_num_sines << std::endl;
        configFile << "Sparse Probability: " << noiseParams_.sparse_probability << std::endl; 
        configFile << "Max Deviation Magnitude: " << noiseParams_.max_deviation_magnitude << " mm" << std::endl;

        configFile.close();
        std::cout << "Noise configuration saved to: " << configPath << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Saving noise configuration: " << e.what() << std::endl;
    }
}

void GenerationPipeline::saveMachineConfigToFile() const {
    try {
        std::string configPath = (fs::path(uniqueSessionFolder_) / "config" / "machine_config.txt").make_preferred().string();
        std::ofstream configFile(configPath);

        if (!configFile.is_open()) {
            std::cerr << "Warning: Could not save machine configuration file" << std::endl;
            return;
        }

        configFile << "=== Machine Configuration ===" << std::endl;
        configFile << "Position Limits (mm):" << std::endl;
        configFile << "  X: [" << machineConstraints_.min_position.x() << ", " << machineConstraints_.max_position.x() << "]" << std::endl;
        configFile << "  Y: [" << machineConstraints_.min_position.y() << ", " << machineConstraints_.max_position.y() << "]" << std::endl;
        configFile << "  Z: [" << machineConstraints_.min_position.z() << ", " << machineConstraints_.max_position.z() << "]" << std::endl;

        configFile << "Velocity Limits (mm/s): [" << machineConstraints_.max_velocity.transpose() << "]" << std::endl;
        configFile << "Acceleration Limits (mm/s²): [" << machineConstraints_.max_acceleration.transpose() << "]" << std::endl;
        configFile << "Jerk Limits (mm/s³): [" << machineConstraints_.max_jerk.transpose() << "]" << std::endl;
        configFile << "Max Feedrate: " << machineConstraints_.max_feedrate << " mm/min" << std::endl;
        configFile << "Safety Margin: " << machineConstraints_.safety_margin << std::endl;

        configFile << "\nMotion Config:" << std::endl;
        configFile << "Controller Frequency: " << motionConfig_.controllerFrequency << " Hz" << std::endl;
        configFile << "Max Jerk: " << motionConfig_.maxJerk << " mm/s³" << std::endl;
        configFile << "Max Acceleration: " << motionConfig_.maxAcceleration << " mm/s²" << std::endl;
        configFile << "Max Velocity: " << motionConfig_.maxVelocity << " mm/s" << std::endl;

        configFile.close();
        std::cout << "Machine configuration saved to: " << configPath << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Saving machine configuration: " << e.what() << std::endl;
    }
}

std::string GenerationPipeline::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    std::tm local_time;
    localtime_s(&local_time, &time_t);
    ss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}