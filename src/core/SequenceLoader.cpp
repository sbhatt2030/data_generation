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
    , readCursor_(0)
    , filesLoaded_(0) {
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
    return scanAndLoad();
}

bool SequenceLoader::scanAndLoad() {
    buffer_.clear();
    readCursor_ = 0;
    filesLoaded_ = 0;

    std::regex npyPattern(R"((\d+)\.npy)");
    std::vector<std::pair<int, std::string>> numberedFiles;

    try {
        for (const auto& entry : fs::directory_iterator(directoryPath_)) {
            if (!entry.is_regular_file()) continue;
            std::string filename = entry.path().filename().string();
            std::smatch match;
            if (std::regex_match(filename, match, npyPattern)) {
                numberedFiles.push_back({ std::stoi(match[1].str()), entry.path().string() });
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

    std::sort(numberedFiles.begin(), numberedFiles.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    for (const auto& [number, path] : numberedFiles) {
        size_t currentBytes = buffer_.size() * sizeof(Eigen::Vector3d);
        if (currentBytes >= MAX_BUFFER_BYTES) {
            std::cout << "SequenceLoader: 200MB cap reached, stopping at file "
                << number << " (" << filesLoaded_ << " files loaded)" << std::endl;
            break;
        }

        std::vector<Eigen::Vector3d> fileData;
        if (!loadNpyFile(path, fileData)) {
            std::cerr << "WARNING: Skipping " << fs::path(path).filename().string()
                << ": " << lastError_ << std::endl;
            continue;
        }

        buffer_.insert(buffer_.end(), fileData.begin(), fileData.end());
        filesLoaded_++;
    }

    if (buffer_.empty()) {
        setError("No valid data loaded from directory: " + directoryPath_);
        return false;
    }

    std::cout << "SequenceLoader: Loaded " << filesLoaded_ << " files, "
        << buffer_.size() << " samples ("
        << (buffer_.size() * sizeof(Eigen::Vector3d)) / (1024 * 1024)
        << " MB) into RAM" << std::endl;

    return true;
}

std::vector<Eigen::Vector3d> SequenceLoader::loadNextChunk() {
    if (readCursor_ >= buffer_.size()) {
        return {};
    }

    size_t remaining = buffer_.size() - readCursor_;
    size_t chunkSize = std::min(remaining, CHUNK_SIZE);

    std::vector<Eigen::Vector3d> chunk(
        buffer_.begin() + readCursor_,
        buffer_.begin() + readCursor_ + chunkSize);

    readCursor_ += chunkSize;

    std::cout << "SequenceLoader: Served chunk of " << chunkSize
        << " samples (cursor " << readCursor_ << "/" << buffer_.size() << ")" << std::endl;

    return chunk;
}

bool SequenceLoader::hasMoreData() const {
    return readCursor_ < buffer_.size();
}

void SequenceLoader::reset() {
    readCursor_ = 0;
    std::cout << "SequenceLoader: Reset cursor to beginning" << std::endl;
}

bool SequenceLoader::loadNpyFile(const std::string& filepath,
    std::vector<Eigen::Vector3d>& outData) {
    outData.clear();

    try {
        cnpy::NpyArray arr = cnpy::npy_load(filepath);

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

        if (rows == 0) {
            setError("File contains no data");
            return false;
        }

        double* data = arr.data<double>();
        outData.reserve(rows);

        for (size_t i = 0; i < rows; ++i) {
            outData.emplace_back(
                data[i * 3 + 0],
                data[i * 3 + 1],
                data[i * 3 + 2]);
        }

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
    for (size_t i = 0; i < data.size(); ++i) {
        if (!std::isfinite(data[i].x()) ||
            !std::isfinite(data[i].y()) ||
            !std::isfinite(data[i].z())) {
            setError("Invalid data at sample " + std::to_string(i) + ": contains NaN or Inf");
            return false;
        }
    }
    return true;
}

void SequenceLoader::setError(const std::string& error) {
    lastError_ = error;
}