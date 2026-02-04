#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

/**
 * Loads sequential .npy files containing deviation or VFF data
 *
 * Expected file format:
 * - Files named: 0.npy, 1.npy, 2.npy, ...
 * - Shape: (N, 3) where N <= 8000
 * - Data: [x, y, z] samples in millimeters
 */
class SequenceLoader {
public:
    /**
     * Constructor
     * @param directoryPath Path to directory containing numbered .npy files
     */
    explicit SequenceLoader(const std::string& directoryPath);

    /**
     * Initialize loader - scan directory and validate files
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * Load next chunk of data (up to 8000 samples)
     * @return Vector of 3D samples, empty if no more data
     */
    std::vector<Eigen::Vector3d> loadNextChunk();

    /**
     * Check if more data available
     * @return true if more chunks can be loaded
     */
    bool hasMoreData() const;

    /**
     * Reset to beginning of sequence
     */
    void reset();

    /**
     * Get total number of files found
     */
    size_t getFileCount() const { return fileList_.size(); }

    /**
     * Get last error message
     */
    const std::string& getLastError() const { return lastError_; }

private:
    std::string directoryPath_;
    std::vector<std::string> fileList_;  // Sorted list of .npy file paths
    size_t currentFileIndex_;
    std::string lastError_;

    /**
     * Scan directory for .npy files and sort numerically
     */
    bool scanDirectory();

    /**
     * Load single .npy file
     * @param filepath Path to .npy file
     * @param outData Output vector of samples
     * @return true if load successful
     */
    bool loadNpyFile(const std::string& filepath, std::vector<Eigen::Vector3d>& outData);

    /**
     * Validate loaded data
     * @param data Data to validate
     * @return true if valid
     */
    bool validateData(const std::vector<Eigen::Vector3d>& data);

    /**
     * Set error message
     */
    void setError(const std::string& error);
};