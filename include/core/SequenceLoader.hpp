#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

/**
 * Loads sequential .npy files containing deviation or VFF data.
 * All files in the directory are loaded into RAM during initialize(),
 * up to the 200MB cap. loadNextChunk() serves data from RAM only.
 *
 * Expected file format:
 * - Files named: 0.npy, 1.npy, 2.npy, ...
 * - Shape: (N, 3) where N is any positive number of samples
 * - Data: [x, y, z] samples in millimeters
 */
class SequenceLoader {
public:
    explicit SequenceLoader(const std::string& directoryPath);

    bool initialize();

    std::vector<Eigen::Vector3d> loadNextChunk();

    bool hasMoreData() const;

    void reset();

    size_t getFileCount() const { return filesLoaded_; }
    size_t getTotalSamples() const { return buffer_.size(); }

    const std::string& getLastError() const { return lastError_; }

private:
    static constexpr size_t MAX_BUFFER_BYTES = 200ULL * 1024 * 1024;
    static constexpr size_t CHUNK_SIZE = 8000;

    std::string directoryPath_;
    std::vector<Eigen::Vector3d> buffer_;
    size_t readCursor_;
    size_t filesLoaded_;
    std::string lastError_;

    bool scanAndLoad();
    bool loadNpyFile(const std::string& filepath, std::vector<Eigen::Vector3d>& outData);
    bool validateData(const std::vector<Eigen::Vector3d>& data);
    void setError(const std::string& error);
};