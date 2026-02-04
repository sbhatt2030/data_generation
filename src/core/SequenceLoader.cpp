#include "core/SequenceLoader.hpp"
#include "core/cnpy.h"
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <regex>
#include <cmath>

namespace fs = std::filesystem;

SequenceLoader::SequenceLoader(const std::string& directoryPath)
    : directoryPath_(directoryPath)
    , currentFileIndex_(0) {
}

bool SequenceLoader::initialize() {
    if (directoryPath_.empty()) {
        setError("Directory path is empty");
        return false;
    }

    if (!fs::exists(directoryPath_)) {
        setError("Directory does not exist: " + directoryPath_);
        return false;
    }

    if (!fs::is_directory(directoryPath_)) {
        setError("Path is not a directory: " + directoryPath_);
        return false;
    }

    return scanDirectory();
}

bool SequenceLoader::scanDirectory() {
    fileList_.clear();

    // Regex to match numbered .npy files (0.npy, 1.npy, etc.)
    std::regex npyPattern(R"((\d+)\.npy)");

    std::vector<std::pair<int, std::string>> numberedFiles;

    try {
        for (const auto& entry : fs::directory_iterator(directoryPath_)) {
            if (!entry.is_regular_file()) continue;

            std::string filename = entry.path().filename().string();
            std::smatch match;

            if (std::regex_match(filename, match, npyPattern)) {
                int fileNumber = std::stoi(match[1].str());
                numberedFiles.push_back({ fileNumber, entry.path().string() });
            }
        }
    }
    catch (const std::exception& e) {
        setError("Error scanning directory: " + std::string(e.what()));
        return false;
    }

    if (numberedFiles.empty()) {
        setError("No .npy files found in directory: " + directoryPath_);
        return false;
    }

    // Sort by file number
    std::sort(numberedFiles.begin(), numberedFiles.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    // Extract file paths in sorted order
    for (const auto& [number, path] : numberedFiles) {
        fileList_.push_back(path);
    }

    std::cout << "SequenceLoader: Found " << fileList_.size()
        << " .npy files in " << directoryPath_ << std::endl;

    currentFileIndex_ = 0;
    return true;
}

std::vector<Eigen::Vector3d> SequenceLoader::loadNextChunk() {
    std::vector<Eigen::Vector3d> chunk;

    if (currentFileIndex_ >= fileList_.size()) {
        return chunk; // Empty - no more files
    }

    const std::string& filepath = fileList_[currentFileIndex_];

    if (!loadNpyFile(filepath, chunk)) {
        std::cerr << "ERROR: Failed to load " << filepath << ": " << lastError_ << std::endl;
        currentFileIndex_++; // Skip failed file
        return {}; // Return empty on error
    }

    currentFileIndex_++;

    std::cout << "Loaded chunk from " << fs::path(filepath).filename().string()
        << ": " << chunk.size() << " samples" << std::endl;

    return chunk;
}

bool SequenceLoader::loadNpyFile(const std::string& filepath,
    std::vector<Eigen::Vector3d>& outData) {
    outData.clear();

    try {
        // Load .npy file
        cnpy::NpyArray arr = cnpy::npy_load(filepath);

        // Validate shape
        if (arr.shape.size() != 2) {
            setError("Expected 2D array, got " + std::to_string(arr.shape.size()) + "D");
            return false;
        }

        size_t rows = arr.shape[0];
        size_t cols = arr.shape[1];

        if (cols != 3) {
            setError("Expected 3 columns (x,y,z), got " + std::to_string(cols));
            return false;
        }

        if (rows > 8000) {
            setError("Too many rows: " + std::to_string(rows) + " (max 8000)");
            return false;
        }

        if (rows == 0) {
            setError("File contains no data");
            return false;
        }

        // Convert to Eigen vectors
        double* data = arr.data<double>();
        outData.reserve(rows);

        for (size_t i = 0; i < rows; ++i) {
            Eigen::Vector3d sample(
                data[i * 3 + 0],  // x
                data[i * 3 + 1],  // y
                data[i * 3 + 2]   // z
            );
            outData.push_back(sample);
        }

        // Validate data
        if (!validateData(outData)) {
            outData.clear();
            return false;
        }

        return true;
    }
    catch (const std::exception& e) {
        setError("Exception loading file: " + std::string(e.what()));
        return false;
    }
}

bool SequenceLoader::validateData(const std::vector<Eigen::Vector3d>& data) {
    // Check for NaN or Inf values
    for (size_t i = 0; i < data.size(); ++i) {
        const auto& sample = data[i];

        if (!std::isfinite(sample.x()) ||
            !std::isfinite(sample.y()) ||
            !std::isfinite(sample.z())) {

            setError("Invalid data at sample " + std::to_string(i) +
                ": contains NaN or Inf");
            return false;
        }
    }

    return true;
}

bool SequenceLoader::hasMoreData() const {
    return currentFileIndex_ < fileList_.size();
}

void SequenceLoader::reset() {
    currentFileIndex_ = 0;
    std::cout << "SequenceLoader: Reset to beginning of sequence" << std::endl;
}

void SequenceLoader::setError(const std::string& error) {
    lastError_ = error;
}