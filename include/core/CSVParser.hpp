#pragma once

#include "core/CNCExperimentRunner.hpp"
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

/**
 * CSV Parser for CNC experiment configurations
 * Parses CSV files into ExperimentConfig structures for batch processing
 */
class CSVParser {
public:
    /**
     * Parse CSV file into experiment configurations
     * @param csvFilePath Path to CSV file
     * @return Vector of ExperimentConfig objects
     */
    static std::vector<ExperimentConfig> parseCSV(const std::string& csvFilePath);

    /**
     * Validate a single experiment configuration
     * @param config ExperimentConfig to validate
     * @return true if configuration is valid
     */
    static bool validateConfig(const ExperimentConfig& config);

    /**
     * Get last parse error message
     * @return Error message from last parse operation
     */
    static const std::string& getLastError();

private:
    static std::string lastError_;

    /**
     * Parse a single CSV row into ExperimentConfig
     * @param row CSV row data
     * @param rowNumber Row number for error reporting
     * @param config Output ExperimentConfig
     * @return true if parsing successful
     */
    static bool parseRow(const std::vector<std::string>& row, int rowNumber, ExperimentConfig& config);

    /**
     * Parse CSV line into tokens
     * @param line CSV line
     * @return Vector of tokens
     */
    static std::vector<std::string> parseCSVLine(const std::string& line);

    /**
     * Trim whitespace from string
     * @param str String to trim
     * @return Trimmed string
     */
    static std::string trim(const std::string& str);

    /**
     * Convert string to TrajectoryType enum
     * @param str String representation
     * @return TrajectoryType enum value
     */
    static TrajectoryType parseTrajectoryType(const std::string& str);

    /**
     * Convert string to KinematicNoiseType enum
     * @param str String representation
     * @return KinematicNoiseType enum value
     */
    static KinematicNoiseType parseNoiseType(const std::string& str);

    /**
     * Convert string to VffType enum
     * @param str String representation
     * @return VffType enum value
     */
    static VffType parseVffType(const std::string& str);

    /**
     * Convert string to boolean
     * @param str String representation ("true", "false", "1", "0")
     * @return Boolean value
     */
    static bool parseBool(const std::string& str);

    /**
     * Set error message
     * @param error Error message
     */
    static void setError(const std::string& error);

    static void setConfigDefaults(ExperimentConfig& config);
};