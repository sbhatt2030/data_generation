#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

/**
 * Loads a single .npy file containing deviation or VFF data.
 * The entire file is loaded into RAM during initialize().
 * loadNextChunk() serves data from RAM only — no disk I/O at runtime.
 *
 * Expected file format:
 * - Shape: (N, 3) where N is the total number of samples
 * - Data: [x, y, z] samples in millimeters
 * - Dtype: float32 or float64 (float32 is upcast to double on load)
 */
class SequenceLoader {
public:
    explicit SequenceLoader(const std::string& filePath);

    bool initialize();

    std::vector<Eigen::Vector3d> loadNextChunk();

    bool hasMoreData() const;

    void reset();

    size_t getTotalSamples() const { return buffer_.size(); }

    const std::string& getLastError() const { return lastError_; }

private:
    static constexpr size_t CHUNK_SIZE = 8000;

    std::string filePath_;
    std::vector<Eigen::Vector3d> buffer_;
    size_t readCursor_;
    std::string lastError_;

    bool loadNpyFile(const std::string& filepath, std::vector<Eigen::Vector3d>& outData);
    bool validateData(const std::vector<Eigen::Vector3d>& data);
    void setError(const std::string& error);
};