#pragma once
#ifndef EXTRACTION_PIPELINE_HPP
#define EXTRACTION_PIPELINE_HPP

#include "core/MotionService.h"  // Enhanced MotionService with atomic counters
#include "core/system_constants.hpp"
#include <string>
#include <vector>
#include <set>
#include <fstream>
#include<iostream>
#include <iomanip>
#include <filesystem>

/**
 * Configuration constants for extraction pipeline
 */
namespace ExtractionConfig {
    constexpr size_t FIXED_BUFFER_SIZE = SystemConstants::Buffers::EXTRACTION_BUFFER_SIZE;
    constexpr size_t LOGGING_INTERVAL = SystemConstants::Buffers::EXTRACTION_LOGGING_INTERVAL;
    constexpr long SMR_READ_TIMEOUT = 0;   // Keep this - it's SMR-specific
}

/**
 * Motion detection thresholds (from existing working system)
 */
namespace MotionThresholds {
    constexpr double VELOCITY_THRESHOLD = 1e-6;           // mm/s - global motion detection
    constexpr double ACCELERATION_THRESHOLD = 1e-3;       // mm/s² - motion transition detection
    constexpr double AXIS_VELOCITY_THRESHOLD = 1e-4;      // mm/s - per-axis motion detection
    constexpr double AXIS_ACCELERATION_THRESHOLD = 1e-2;  // mm/s² - per-axis motion detection
}

/**
 * Enhanced data point structure matching SMR format
 */
struct ExtractedDataPoint {
    double pos_x, pos_y, pos_z;
    double dev_x, dev_y, dev_z;
    double vff_x, vff_y, vff_z;
    double e_enc_x, e_enc_y, e_enc_z;
    double e_scale_x, e_scale_y, e_scale_z;
    int line_number;
    bool has_motion;

    ExtractedDataPoint()
        : pos_x(0), pos_y(0), pos_z(0)
        , dev_x(0), dev_y(0), dev_z(0)
        , vff_x(0), vff_y(0), vff_z(0)
        , e_enc_x(0), e_enc_y(0), e_enc_z(0)
        , e_scale_x(0), e_scale_y(0), e_scale_z(0)
        , line_number(-1)
        , has_motion(false) {
    }
};

/**
 * Statistics for centralized monitoring
 */
struct ExtractionStats {
    size_t totalPointsRead = 0;
    size_t totalPointsKept = 0;
    size_t readAttempts = 0;
    size_t readFailures = 0;
    size_t csvFilesWritten = 0;
    size_t csvWriteErrors = 0;
    size_t currentBufferSize = 0;
    size_t motionPointsInBuffer = 0;
    size_t pointsWithMotion = 0;
    size_t positionSpikeCount = 0;
    int currentLineNumber = -1;
    bool extractionComplete = false;
    std::string status = "Initializing";
};

/**
 * Simplified Sequential Extraction Pipeline
 * - Fills 8000-point buffers with all data
 * - Dumps only buffers that contain motion points
 * - Tracks line number transitions for termination
 * - NEW: Supports external termination control
 */
class ExtractionPipeline {
public:
    /**
     * Constructor
     */
    ExtractionPipeline();

    /**
     * Initialize with enhanced MotionService and session folder
     * @param motionService Enhanced MotionService with atomic counters
     * @param sessionFolder Session folder created by GenerationPipeline
     * @param expectedLineNumbers Set of line numbers to expect (for completion detection)
     * @return true if initialization successful
     */
    bool initialize(MotionService* motionService,
        const std::string& sessionFolder,
        const std::set<int>& expectedLineNumbers, 
        const std::string& spikeLogPath = "");

    /**
     * Main processing - handles SMR reads, filtering, and CSV writes
     * Called once per main loop cycle (100Hz)
     */
    void processOneCycle();

    /**
     * Check if extraction should continue
     * NEW: Now checks external stop condition first, then existing logic
     * @return true if more data expected or buffers have data
     */
    bool shouldContinueExtracting() const;

    /**
     * Force flush any remaining data and close files
     * Called during shutdown
     */
    void finalFlush();

    /**
     * Get current statistics for centralized monitoring
     * @return Current extraction statistics
     */
    ExtractionStats getStats() const;

    /**
     * Manual status logging
     */
    void logStatus() const;

    void setComplete() {
        stats_.extractionComplete = true;
        stats_.status = "Complete (CNC Done)";
    }


    // Status getters for centralized monitoring
    size_t getTotalPointsRead() const { return stats_.totalPointsRead; }
    size_t getCurrentBufferSize() const { return currentBuffer_.size(); }
    bool isExtractionComplete() const { return stats_.extractionComplete; }
    int getCurrentLineNumber() const { return currentLineNumber_; }
    size_t getPositionSpikeCount() const { return positionSpikeCount_; }

    bool reconfigure(MotionService* motionService,
        const std::string& sessionFolder,
        const std::set<int>& expectedLineNumbers,
        const std::string& spikeLogPath = "");

    // NEW: Reset experiment-specific state
    void resetExperimentState();

    // NEW: Reset position tracking
    void resetPositionTracking();


private:
    // Dependencies
    MotionService* motionService_;        // Enhanced MotionService with atomic counters
    std::string sessionFolder_;          // Session folder for output
    std::string resultsFolder_;          // Results subfolder for CSV files
    bool basicInitializationDone_ = false;



    // Single buffer system (always 8000 points)
    std::vector<ExtractedDataPoint> currentBuffer_;

    // State tracking
    bool bufferHasMotion_;              // Flag: buffer contains at least one motion point
    int currentLineNumber_;             // Current line number (-1 if none seen yet)

    // File management (static counter for single thread)
    int fileCounter_;            // Static counter for sequential file naming

    // Statistics and monitoring
    mutable ExtractionStats stats_;
    mutable size_t lastLogPoint_;        // Last point count when logged

    size_t positionSpikeCount_ = 0;
    std::string spikeLogPath_;

    // Core processing methods
    void readAllAvailablePoints();
    bool readSinglePoint(ExtractedDataPoint& point);
    void addPointToBuffer(const ExtractedDataPoint& point);
    bool shouldDumpBuffer() const;
    void dumpBufferToFile();
    void resetBuffer();

    // File operations
    std::string generateSequentialFilename();
    bool writeBufferToCSV(const std::string& filename);

    void updateStats();
    void checkExtractionCompletion();

    bool hasPreviousPosition_ = false;
    double prevPosX_ = 0.0, prevPosY_ = 0.0, prevPosZ_ = 0.0;

    std::string getCurrentTimestamp() const;
    void logPositionSpike(double deltaX, double deltaY, double deltaZ, int lineNumber);
};
#endif // EXTRACTION_PIPELINE_HPP