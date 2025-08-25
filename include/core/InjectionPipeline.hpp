#pragma once

#include "GenerationPipeline.hpp"
#include "core/MotionService.h"  // Enhanced MotionService with atomic counters
#include "core/system_constants.hpp"
#include <deque>
#include <string>

/**
 * Configuration constants for injection pipeline
 */
namespace InjectionConfig {
    constexpr size_t BUFFER_CAPACITY = SystemConstants::Buffers::INJECTION_BUFFER_CAPACITY;
    constexpr size_t REFILL_TRIGGER = SystemConstants::Buffers::INJECTION_REFILL_TRIGGER;
    constexpr size_t LOGGING_INTERVAL = SystemConstants::Buffers::INJECTION_LOGGING_INTERVAL;
    constexpr long SMR_WRITE_TIMEOUT = 0;  // Keep this - it's SMR-specific
}

/**
 * Statistics for centralized monitoring
 */
struct InjectionStats {
    size_t totalPointsReceived = 0;      // Points received from generation pipeline
    size_t totalPointsWritten = 0;       // Points successfully written to SMR
    size_t writeAttempts = 0;            // Total write attempts to SMR
    size_t writeFailures = 0;            // Failed writes (SMR buffer full)
    size_t currentBufferSize = 0;        // Current points in injection buffer
    size_t trajectoriesGenerated = 0;    // Total trajectories requested from generator
    double bufferFillLevel = 0.0;        // 0.0-1.0 buffer fill percentage
    std::string status = "Initializing";  // Current status description
};

/**
 * Fresh Injection Pipeline - Direct MotionService communication
 * No compatibility with old DataInjectionSystem - clean implementation
 */
class InjectionPipeline {
public:
    /**
     * Constructor
     */
    InjectionPipeline();

    /**
     * Initialize with enhanced MotionService and GenerationPipeline
     * @param motionService Enhanced MotionService with atomic counters
     * @param generator GenerationPipeline for trajectory data
     * @return true if initialization successful
     */
    bool initialize(MotionService* motionService, GenerationPipeline* generator);

    /**
     * Main processing - handles refill and SMR writes
     * Called once per main loop cycle (100Hz)
     */
    void processOneCycle();

    /**
     * Add points directly to buffer (alternative to generation)
     * @param points Trajectory points to add
     * @return true if successful
     */
    bool addPoints(const std::vector<InputDataPoint>& points);

    /**
     * Check if buffer needs refilling
     * @return true if buffer < 50% full
     */
    bool needsRefill() const { return buffer_.size() < InjectionConfig::REFILL_TRIGGER; }

    /**
     * Get current statistics for centralized monitoring
     * @return Current injection statistics
     */
    InjectionStats getStats() const;

    /**
     * Manual status logging
     */
    void logStatus() const;

    /**
    * Force flush any remaining data and close files
    * Called during shutdown
    */
    void finalFlush();

    // Status getters for centralized monitoring
    size_t getBufferSize() const { return buffer_.size(); }
    size_t getTotalPointsWritten() const { return stats_.totalPointsWritten; }
    size_t getWriteFailures() const { return stats_.writeFailures; }
    double getBufferFillLevel() const { return static_cast<double>(buffer_.size()) / InjectionConfig::BUFFER_CAPACITY; }
    static void resetLineCounter();
    static int getNextLineNumber();

private:
    // Dependencies (passed by reference for clean architecture)
    MotionService* motionService_;         // Enhanced MotionService with atomic counters
    GenerationPipeline* generator_;        // Fresh GenerationPipeline
    static std::atomic<int> globalLineCounter_;
    static std::atomic<bool> countingUp_;
    static constexpr int MAX_LINE_NUMBER = 1000000;


    // Internal buffer (16k capacity)
    std::deque<InputDataPoint> buffer_;

    // Statistics and monitoring
    mutable InjectionStats stats_;
    mutable size_t lastLogPoint_;          // Last point count when logged

    // Core processing methods
    void refillBufferIfNeeded();
    void writeMaxPointsToSMR();
    bool writeSinglePointToSMR();
    void updateStats();
};