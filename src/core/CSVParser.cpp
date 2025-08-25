#include "core/CSVParser.hpp"
#include <iostream>
#include <algorithm>
#include <cctype>
#include <optional>

// Static member definition
std::string CSVParser::lastError_;

std::vector<ExperimentConfig> CSVParser::parseCSV(const std::string& csvFilePath) {
    std::vector<ExperimentConfig> experiments;
    lastError_.clear();

    std::ifstream file(csvFilePath);
    if (!file.is_open()) {
        setError("Cannot open CSV file: " + csvFilePath);
        return experiments;
    }

    std::string line;
    int rowNumber = 0;

    // Skip header row
    if (std::getline(file, line)) {
        rowNumber++;
        std::cout << "Skipping header: " << line << std::endl;
    }

    // Parse data rows
    while (std::getline(file, line)) {
        rowNumber++;

        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Parse CSV line into tokens
        std::vector<std::string> tokens = parseCSVLine(line);

        // Expected format: 23 columns
        if (tokens.size() != 23) {
            setError("Row " + std::to_string(rowNumber) + ": Expected 23 columns, got " + std::to_string(tokens.size()));
            continue;
        }

        // Parse row into ExperimentConfig
        ExperimentConfig config;
        if (parseRow(tokens, rowNumber, config)) {
            if (validateConfig(config)) {
                experiments.push_back(config);
                std::cout << "Parsed experiment: " << config.experimentId << std::endl;
            }
            else {
                std::cerr << "Validation failed for row " << rowNumber << ": " << lastError_ << std::endl;
            }
        }
        else {
            std::cerr << "Parse failed for row " << rowNumber << ": " << lastError_ << std::endl;
        }
    }

    file.close();
    std::cout << "Successfully parsed " << experiments.size() << " experiments from " << csvFilePath << std::endl;
    return experiments;
}

bool CSVParser::parseRow(const std::vector<std::string>& row, int rowNumber, ExperimentConfig& config) {
    try {
        // Expect exactly 23 columns
        if (row.size() != 23) {
            setError("Row " + std::to_string(rowNumber) + ": Expected 23 columns, got " + std::to_string(row.size()));
            return false;
        }

        // Helper functions for safe parsing
        auto parseDouble = [](const std::string& str, double defaultValue = 0.0) -> double {
            if (str.empty()) return defaultValue;
            try {
                return std::stod(str);
            }
            catch (const std::exception&) {
                return defaultValue;
            }
            };

        auto parseInt = [](const std::string& str, int defaultValue = 0) -> int {
            if (str.empty()) return defaultValue;
            try {
                return std::stoi(str);
            }
            catch (const std::exception&) {
                return defaultValue;
            }
            };

        auto parseUInt = [](const std::string& str, unsigned int defaultValue = 0) -> unsigned int {
            if (str.empty()) return defaultValue;
            try {
                return static_cast<unsigned int>(std::stoul(str));
            }
            catch (const std::exception&) {
                return defaultValue;
            }
            };

        // ================================================================
        // Basic Experiment Info (Columns 0-2)
        // ================================================================

        // Column 0: experiment_id
        config.experimentId = trim(row[0]);
        if (config.experimentId.empty()) {
            setError("Row " + std::to_string(rowNumber) + ": experiment_id cannot be empty");
            return false;
        }

        // Column 1: family_id
        config.familyId = trim(row[1]);
        if (config.familyId.empty()) {
            config.familyId = "default_family";
        }

        // Column 2: output_directory
        config.outputDirectory = trim(row[2]);
        if (config.outputDirectory.empty()) {
            config.outputDirectory = "C:\\data\\default";
        }

        // ================================================================
        // G-code Generation Parameters (Columns 3-4)
        // ================================================================

        // Column 3: num_trajectories
        config.gcodeParams.num_trajectories = parseInt(trim(row[3]), 30);
        if (config.gcodeParams.num_trajectories <= 0 || config.gcodeParams.num_trajectories > 1000) {
            setError("Row " + std::to_string(rowNumber) + ": num_trajectories must be between 1 and 1000");
            return false;
        }

        // Column 4: trajectory_type
        int trajType = parseInt(trim(row[4]), 2);
        switch (trajType) {
        case 0: config.gcodeParams.trajectory_type = TrajectoryType::LINEAR_ONLY; break;
        case 1: config.gcodeParams.trajectory_type = TrajectoryType::CIRCULAR_ONLY; break;
        case 2: config.gcodeParams.trajectory_type = TrajectoryType::MIXED; break;
        default:
            setError("Row " + std::to_string(rowNumber) + ": Invalid trajectory_type: " + std::to_string(trajType));
            return false;
        }

        // ================================================================
        // Noise Parameters (Columns 5-12)
        // ================================================================

        // Column 5: noise_type
        int noiseType = parseInt(trim(row[5]), 0);
        switch (noiseType) {
        case 0: config.noiseType = KinematicNoiseType::SMOOTH_GAUSSIAN_BANDPASS; break;
        case 1: config.noiseType = KinematicNoiseType::SUM_OF_SINUSOIDS; break;
        case 2: config.noiseType = KinematicNoiseType::SPARSE_INJECTION; break;
        case 3: config.noiseType = KinematicNoiseType::NO_NOISE; break;
        default:
            setError("Row " + std::to_string(rowNumber) + ": Invalid noise_type: " + std::to_string(noiseType));
            return false;
        }

        // Column 6: noise_min_amplitude
        config.noiseParams.min_amplitude = parseDouble(trim(row[6]), 0.001);
        if (config.noiseParams.min_amplitude < 0.0 || config.noiseParams.min_amplitude > 0.9) {
            setError("Row " + std::to_string(rowNumber) + ": noise_min_amplitude must be between 0.0 and 0.9 mm");
            return false;
        }

        // Column 7: noise_max_amplitude
        config.noiseParams.max_amplitude = parseDouble(trim(row[7]), 0.01);
        if (config.noiseParams.max_amplitude < 0.0 || config.noiseParams.max_amplitude > 0.9) {
            setError("Row " + std::to_string(rowNumber) + ": noise_max_amplitude must be between 0.0 and 0.9 mm");
            return false;
        }

        // Validate amplitude range
        if (config.noiseParams.min_amplitude > config.noiseParams.max_amplitude) {
            setError("Row " + std::to_string(rowNumber) + ": noise_min_amplitude must be <= noise_max_amplitude");
            return false;
        }

        // Column 8: noise_min_freq
        config.noiseParams.min_frequency = parseDouble(trim(row[8]), 0.5);
        if (config.noiseParams.min_frequency <= 0.0) {
            setError("Row " + std::to_string(rowNumber) + ": noise_min_freq must be > 0.0 Hz");
            return false;
        }

        // Column 9: noise_max_freq
        config.noiseParams.max_frequency = parseDouble(trim(row[9]), 50.0);
        if (config.noiseParams.max_frequency > 2000.0) {
            setError("Row " + std::to_string(rowNumber) + ": noise_max_freq must be <= 2000 Hz (Nyquist limit)");
            return false;
        }

        // Validate frequency range
        if (config.noiseParams.min_frequency > config.noiseParams.max_frequency) {
            setError("Row " + std::to_string(rowNumber) + ": noise_min_freq must be <= noise_max_freq");
            return false;
        }

        // Column 10: noise_min_sines
        config.noiseParams.min_num_sines = parseInt(trim(row[10]), 3);
        if (config.noiseParams.min_num_sines < 1) {
            setError("Row " + std::to_string(rowNumber) + ": noise_min_sines must be >= 1");
            return false;
        }

        // Column 11: noise_max_sines
        config.noiseParams.max_num_sines = parseInt(trim(row[11]), 8);
        if (config.noiseParams.max_num_sines > 20) {
            setError("Row " + std::to_string(rowNumber) + ": noise_max_sines must be <= 20");
            return false;
        }

        // Validate sine count range
        if (config.noiseParams.min_num_sines > config.noiseParams.max_num_sines) {
            setError("Row " + std::to_string(rowNumber) + ": noise_min_sines must be <= noise_max_sines");
            return false;
        }

        // Column 12: noise_sparse_prob
        config.noiseParams.sparse_probability = parseDouble(trim(row[12]), 0.03);
        if (config.noiseParams.sparse_probability < 0.0 || config.noiseParams.sparse_probability > 1.0) {
            setError("Row " + std::to_string(rowNumber) + ": noise_sparse_prob must be between 0.0 and 1.0");
            return false;
        }

        // ================================================================
        // VFF Parameters (Columns 13-18)  
        // ================================================================

        // Column 13: vff_type
        int vffType = parseInt(trim(row[13]), 0);
        switch (vffType) {
        case 0: config.vffConfig.vffType = VffType::SMOOTH_GAUSSIAN; break;
        case 1: config.vffConfig.vffType = VffType::SMOOTH_GAUSSIAN_DC_SHIFT; break;
        case 2: config.vffConfig.vffType = VffType::SPARSE_VFF; break;
        case 3: config.vffConfig.vffType = VffType::NO_VFF;
                config.vffConfig.useVffGenerator = false; 
                break;
        default:
            setError("Row " + std::to_string(rowNumber) + ": Invalid vff_type: " + std::to_string(vffType));
            return false;
        }

        // Column 14: vff_min_dc
        config.vffConfig.vffParams.min_dc_shift = parseDouble(trim(row[14]), -5.0);

        // Column 15: vff_max_dc
        config.vffConfig.vffParams.max_dc_shift = parseDouble(trim(row[15]), 5.0);

        // Validate DC shift range
        if (config.vffConfig.vffParams.min_dc_shift > config.vffConfig.vffParams.max_dc_shift) {
            setError("Row " + std::to_string(rowNumber) + ": vff_min_dc must be <= vff_max_dc");
            return false;
        }

        // Column 16: vff_max_amplitude
        config.vffConfig.vffParams.max_amplitude = parseDouble(trim(row[16]), 10.0);
        if (config.vffConfig.vffParams.max_amplitude < 0.0) {
            setError("Row " + std::to_string(rowNumber) + ": vff_max_amplitude must be >= 0.0");
            return false;
        }

        // Column 17: vff_max_freq (NEW)
        config.vffConfig.vffParams.max_frequency = parseDouble(trim(row[17]), 50.0);
        if (config.vffConfig.vffParams.max_frequency > 2000.0) {
            setError("Row " + std::to_string(rowNumber) + ": vff_max_freq must be <= 2000 Hz (Nyquist limit)");
            return false;
        }

        // Column 18: vff_sparse_prob
        config.vffConfig.vffParams.sparse_probability = parseDouble(trim(row[18]), 0.02);
        if (config.vffConfig.vffParams.sparse_probability < 0.0 || config.vffConfig.vffParams.sparse_probability > 1.0) {
            setError("Row " + std::to_string(rowNumber) + ": vff_sparse_prob must be between 0.0 and 1.0");
            return false;
        }

        // ================================================================
        // Seeds (Columns 19-22)
        // ================================================================

        // Column 19: master_seed
        unsigned int masterSeed = parseUInt(trim(row[19]), 0);
        if (masterSeed != 0) {
            config.seedConfig.masterSeed = masterSeed;
        }

        // Column 20: gcode_seed
        unsigned int gcodeSeed = parseUInt(trim(row[20]), 0);
        if (gcodeSeed != 0) {
            config.seedConfig.gcodeGeneratorSeed = gcodeSeed;
        }

        // Column 21: noise_seed
        unsigned int noiseSeed = parseUInt(trim(row[21]), 0);
        if (noiseSeed != 0) {
            config.seedConfig.noiseGeneratorSeed = noiseSeed;
        }

        // Column 22: vff_seed
        unsigned int vffSeed = parseUInt(trim(row[22]), 0);
        if (vffSeed != 0) {
            config.seedConfig.vffGeneratorSeed = vffSeed;
        }

        // ================================================================
        // Set remaining defaults and validate
        // ================================================================

        setConfigDefaults(config);
        return true;

    }
    catch (const std::exception& e) {
        setError("Row " + std::to_string(rowNumber) + ": Exception parsing row: " + e.what());
        return false;
    }
}

std::vector<std::string> CSVParser::parseCSVLine(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;

    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }

    return tokens;
}

std::string CSVParser::trim(const std::string& str) {
    if (str.empty()) {
        return str;
    }

    // Find first non-whitespace character
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return ""; // String contains only whitespace
    }

    // Find last non-whitespace character
    size_t end = str.find_last_not_of(" \t\r\n");

    // Extract substring
    return str.substr(start, end - start + 1);
}

TrajectoryType CSVParser::parseTrajectoryType(const std::string& str) {
    int value = std::stoi(str);
    switch (value) {
    case 0: return TrajectoryType::LINEAR_ONLY;
    case 1: return TrajectoryType::CIRCULAR_ONLY;
    case 2: return TrajectoryType::MIXED;
    default:
        throw std::invalid_argument("Invalid trajectory_type: " + str);
    }
}

KinematicNoiseType CSVParser::parseNoiseType(const std::string& str) {
    int value = std::stoi(str);
    switch (value) {
    case 0: return KinematicNoiseType::SMOOTH_GAUSSIAN_BANDPASS;
    case 1: return KinematicNoiseType::SUM_OF_SINUSOIDS;
    case 2: return KinematicNoiseType::SPARSE_INJECTION;
    case 3: return KinematicNoiseType::NO_NOISE;
    default:
        throw std::invalid_argument("Invalid noise_type: " + str);
    }
}

VffType CSVParser::parseVffType(const std::string& str) {
    int value = std::stoi(str);
    switch (value) {
    case 0: return VffType::SMOOTH_GAUSSIAN;
    case 1: return VffType::SMOOTH_GAUSSIAN_DC_SHIFT;
    case 2: return VffType::SPARSE_VFF;
    case 3: return VffType::NO_VFF;
    default:
        throw std::invalid_argument("Invalid vff_type: " + str);
    }
}

bool CSVParser::parseBool(const std::string& str) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "true" || lower == "1" || lower == "yes") {
        return true;
    }
    else if (lower == "false" || lower == "0" || lower == "no") {
        return false;
    }
    else {
        throw std::invalid_argument("Invalid boolean value: " + str);
    }
}

const std::string& CSVParser::getLastError() {
    return lastError_;
}

void CSVParser::setError(const std::string& error) {
    lastError_ = error;
}

void CSVParser::setConfigDefaults(ExperimentConfig& config) {
    // Set noise parameter defaults
    config.noiseParams.max_deviation_magnitude = 0.9;  // Safety limit

    // If frequencies not set, use reasonable defaults
    if (config.noiseParams.min_frequency == 0.0) {
        config.noiseParams.min_frequency = 0.5;
    }
    if (config.noiseParams.max_frequency == 0.0) {
        config.noiseParams.max_frequency = 50.0;
    }

    // If sinusoid counts not set, use defaults
    if (config.noiseParams.min_num_sines == 0) {
        config.noiseParams.min_num_sines = 3;
    }
    if (config.noiseParams.max_num_sines == 0) {
        config.noiseParams.max_num_sines = 8;
    }


    // Set VFF parameter defaults
    config.vffConfig.minAmplitude = 0.1;
    config.vffConfig.minAlpha = 0.01;
    config.vffConfig.maxAlpha = 0.2;
    config.vffConfig.usePerAxisVff = false;
    config.vffConfig.sparseVffProbability = config.vffConfig.vffParams.sparse_probability;
    config.vffConfig.minSparseVffAmplitude = 1.0;
    config.vffConfig.maxSparseVffAmplitude = 25.0;
    config.vffConfig.fixedAmplitudes = { 0.0, 0.0, 0.0 };
    config.vffConfig.fixedAlphas = { 0.0, 0.0, 0.0 };

    // Set G-code generation defaults
    config.gcodeParams.max_trajectory_time = 2.0;
    config.gcodeParams.dwell_time = 0.5;
    config.gcodeParams.linear_probability = 0.6;
    config.gcodeParams.use_dwell_commands = true;
    config.gcodeParams.write_summary_to_file = true;

    // Other defaults
    config.existingGcodeFile = "";
    config.enableDetailedLogging = false;
}

bool CSVParser::validateConfig(const ExperimentConfig& config) {
    // Validate experiment ID
    if (config.experimentId.empty()) {
        setError("experiment_id cannot be empty");
        return false;
    }

    // Validate output directory
    if (config.outputDirectory.empty()) {
        setError("output_directory cannot be empty");
        return false;
    }

    // Validate trajectory count
    if (config.gcodeParams.num_trajectories <= 0 || config.gcodeParams.num_trajectories > 1000) {
        setError("num_trajectories must be between 1 and 1000");
        return false;
    }


    // Validate frequency limits for 4kHz controller (Nyquist = 2kHz)
    if (config.noiseParams.max_frequency > 2000.0) {
        setError("noise_max_freq must be <= 2000 Hz (Nyquist limit for 4kHz controller)");
        return false;
    }

    // Validate frequency range
    if (config.noiseParams.min_frequency <= 0.0 ||
        config.noiseParams.min_frequency > config.noiseParams.max_frequency) {
        setError("Invalid noise frequency range: min=" + std::to_string(config.noiseParams.min_frequency) +
            ", max=" + std::to_string(config.noiseParams.max_frequency));
        return false;
    }

    // Validate sinusoid count range
    if (config.noiseParams.min_num_sines < 1 ||
        config.noiseParams.min_num_sines > config.noiseParams.max_num_sines) {
        setError("Invalid sinusoid count range: min=" + std::to_string(config.noiseParams.min_num_sines) +
            ", max=" + std::to_string(config.noiseParams.max_num_sines));
        return false;
    }

    if (config.noiseParams.max_num_sines > 20) {  // Reasonable upper limit
        setError("Too many sinusoids (" + std::to_string(config.noiseParams.max_num_sines) +
            "), maximum recommended is 20");
        return false;
    }

    // Validate sparse probability
    if (config.noiseParams.sparse_probability < 0.0 || config.noiseParams.sparse_probability > 1.0) {
        setError("Invalid noise sparse probability: " + std::to_string(config.noiseParams.sparse_probability) +
            " (must be between 0.0 and 1.0)");
        return false;
    }

    // Validate VFF parameters if enabled
    if (config.vffConfig.useVffGenerator) {
        if (config.vffConfig.maxAmplitude < 0.0) {
            setError("Invalid VFF amplitude: " + std::to_string(config.vffConfig.maxAmplitude));
            return false;
        }

        // Validate VFF frequency limits for 4kHz controller
        if (config.vffConfig.vffParams.max_frequency > 2000.0) {
            setError("VFF max frequency (" + std::to_string(config.vffConfig.vffParams.max_frequency) +
                "Hz) exceeds Nyquist limit (2000Hz) for 4kHz controller");
            return false;
        }

        if (config.vffConfig.vffParams.sparse_probability < 0.0 ||
            config.vffConfig.vffParams.sparse_probability > 1.0) {
            setError("Invalid VFF sparse probability: " + std::to_string(config.vffConfig.vffParams.sparse_probability));
            return false;
        }

        // Validate DC shift range
        if (config.vffConfig.vffType == VffType::SMOOTH_GAUSSIAN_DC_SHIFT) {
            if (config.vffConfig.vffParams.min_dc_shift > config.vffConfig.vffParams.max_dc_shift) {
                setError("Invalid VFF DC shift range: min=" + std::to_string(config.vffConfig.vffParams.min_dc_shift) +
                    ", max=" + std::to_string(config.vffConfig.vffParams.max_dc_shift));
                return false;
            }
        }
    }

    return true;
}