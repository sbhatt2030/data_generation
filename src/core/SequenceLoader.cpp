#include "core/SequenceLoader.hpp"
#include "core/cnpy.h"
#include <filesystem>
#include <iostream>
#include <cmath>

namespace fs = std::filesystem;

SequenceLoader::SequenceLoader(const std::string& filePath)
    : filePath_(filePath)
    , readCursor_(0) {
}

bool SequenceLoader::initialize() {
    if (filePath_.empty()) {
        setError("File path is empty");
        return false;
    }
    if (!fs::exists(filePath_)) {
        setError("File does not exist: " + filePath_);
        return false;
    }
    if (!fs::is_regular_file(filePath_)) {
        setError("Path is not a regular file: " + filePath_);
        return false;
    }

    buffer_.clear();
    readCursor_ = 0;

    if (!loadNpyFile(filePath_, buffer_)) {
        return false;
    }

    std::cout << "SequenceLoader: Loaded " << buffer_.size() << " samples ("
        << (buffer_.size() * sizeof(Eigen::Vector3d)) / (1024 * 1024)
        << " MB) into RAM from " << fs::path(filePath_).filename().string() << std::endl;

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

        outData.reserve(rows);

        if (arr.word_size == sizeof(float)) {
            // float32 — upcast to double
            float* data = arr.data<float>();
            for (size_t i = 0; i < rows; ++i) {
                outData.emplace_back(
                    static_cast<double>(data[i * 3 + 0]),
                    static_cast<double>(data[i * 3 + 1]),
                    static_cast<double>(data[i * 3 + 2]));
            }
        }
        else if (arr.word_size == sizeof(double)) {
            // float64 — load directly
            double* data = arr.data<double>();
            for (size_t i = 0; i < rows; ++i) {
                outData.emplace_back(
                    data[i * 3 + 0],
                    data[i * 3 + 1],
                    data[i * 3 + 2]);
            }
        }
        else {
            setError("Unsupported dtype: word_size=" + std::to_string(arr.word_size)
                + " (expected 4 for float32 or 8 for float64)");
            return false;
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