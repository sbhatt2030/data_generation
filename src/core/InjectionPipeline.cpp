#include "core/InjectionPipeline.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cstddef>

std::atomic<int> InjectionPipeline::globalLineCounter_{ 0 };
std::atomic<bool> InjectionPipeline::countingUp_{ true };

InjectionPipeline::InjectionPipeline()
    : motionService_(nullptr)
    , generator_(nullptr)
    , lastLogPoint_(0) {

}

bool InjectionPipeline::initialize(MotionService* motionService, GenerationPipeline* generator) {
    if (!motionService) {
        std::cerr << "ERROR: MotionService pointer is null" << std::endl;
        return false;
    }

    if (!generator) {
        std::cerr << "ERROR: GenerationPipeline pointer is null" << std::endl;
        return false;
    }

    motionService_ = motionService;
    generator_ = generator;

    // Initialize statistics
    stats_ = InjectionStats{};
    stats_.status = "Initialized";
    lastLogPoint_ = 0;

    std::cout << "Injection Pipeline initialized successfully" << std::endl;
    std::cout << "  Buffer capacity: " << InjectionConfig::BUFFER_CAPACITY << " points" << std::endl;
    std::cout << "  Refill trigger: " << InjectionConfig::REFILL_TRIGGER << " points ("
        << (100.0 * InjectionConfig::REFILL_TRIGGER / InjectionConfig::BUFFER_CAPACITY) << "%)" << std::endl;
    std::cout << "  Logging interval: " << InjectionConfig::LOGGING_INTERVAL << " points" << std::endl;

    return true;
}

void InjectionPipeline::processOneCycle() {
    // 1. Refill buffer if needed (calls generation internally)
    refillBufferIfNeeded();

    // 2. Write as many points as possible to SMR
    writeMaxPointsToSMR();

    // 3. Update internal statistics
    updateStats();

    // 4. Periodic logging
    if (stats_.totalPointsWritten - lastLogPoint_ >= InjectionConfig::LOGGING_INTERVAL) {
        logStatus();
        lastLogPoint_ = stats_.totalPointsWritten;
    }
}

bool InjectionPipeline::addPoints(const std::vector<InputDataPoint>& points) {
    if (points.empty()) {
        return true; // Nothing to add, but not an error
    }

    // Check if adding these points would exceed buffer capacity
    if (buffer_.size() + points.size() > InjectionConfig::BUFFER_CAPACITY) {
        std::cerr << "ERROR: Adding " << points.size() << " points would exceed buffer capacity ("
            << buffer_.size() << " + " << points.size() << " > "
            << InjectionConfig::BUFFER_CAPACITY << ")" << std::endl;
        return false;
    }

    // Add points to buffer
    buffer_.insert(buffer_.end(), points.begin(), points.end());
    stats_.totalPointsReceived += points.size();

    std::cout << "Added " << points.size() << " points to injection buffer. "
        << "Buffer: " << buffer_.size() << "/" << InjectionConfig::BUFFER_CAPACITY
        << " (" << std::fixed << std::setprecision(1)
        << (100.0 * buffer_.size() / InjectionConfig::BUFFER_CAPACITY) << "%)" << std::endl;

    return true;
}

InjectionStats InjectionPipeline::getStats() const {
    // Update current stats
    const_cast<InjectionPipeline*>(this)->updateStats();
    return stats_;
}

void InjectionPipeline::logStatus() const {
    std::cout << "=== Injection Pipeline Status ===" << std::endl;
    std::cout << "Buffer: " << buffer_.size() << "/" << InjectionConfig::BUFFER_CAPACITY
        << " (" << std::fixed << std::setprecision(1) << (stats_.bufferFillLevel * 100) << "%)" << std::endl;
    std::cout << "Points Written: " << stats_.totalPointsWritten << std::endl;
    std::cout << "Write Success Rate: " << std::fixed << std::setprecision(1);

    if (stats_.writeAttempts > 0) {
        double successRate = 100.0 * (stats_.writeAttempts - stats_.writeFailures) / stats_.writeAttempts;
        std::cout << successRate << "%" << std::endl;
    }
    else {
        std::cout << "N/A" << std::endl;
    }

    std::cout << "Trajectories Generated: " << stats_.trajectoriesGenerated << std::endl;
    std::cout << "Status: " << stats_.status << std::endl;
    std::cout << "=================================" << std::endl;
}

void InjectionPipeline::refillBufferIfNeeded() {
    // Only refill if buffer is below trigger and generator has more commands
    if (!needsRefill()) {
        return; // Buffer doesn't need refilling
    }

    if (!generator_->hasMoreCommands()) {
        if (stats_.status != "Generation Complete") {
            stats_.status = "Generation Complete";
            std::cout << "Generation pipeline has no more commands. Buffer will drain naturally." << std::endl;
        }
        return; // No more data available
    }

    try {
        // Request next trajectory from generation pipeline
        auto trajectoryPoints = generator_->generateNextCommandData();

        if (!trajectoryPoints.empty()) {
            // Add generated points to buffer
            if (addPoints(trajectoryPoints)) {
                stats_.trajectoriesGenerated++;
                stats_.status = "Active Generation";
            }
            else {
                std::cerr << "ERROR: Failed to add generated points to buffer" << std::endl;
                stats_.status = "Buffer Error";
            }
        }
        else {
            std::cerr << "WARNING: Generation pipeline returned empty trajectory" << std::endl;
        }

    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Exception during buffer refill: " << e.what() << std::endl;
        stats_.status = "Generation Error";
    }
}

void InjectionPipeline::writeMaxPointsToSMR() {
    if (buffer_.empty()) {
        return; // No data to write
    }

    size_t pointsWritten = 0;

    // Write as many points as possible
    while (!buffer_.empty()) {
        if (writeSinglePointToSMR()) {
            pointsWritten++;
        }
        else {
            // Stop on 
            // failure - don't retry in same cycle
            break;
        }
    }

    // Update status based on write results
    if (pointsWritten > 0) {
        stats_.status = "Writing Successfully";
    }
    else if (!buffer_.empty()) {
        stats_.status = "SMR Write Failed";
    }
}

bool InjectionPipeline::writeSinglePointToSMR() {
    if (buffer_.empty()) {
        return false; // No data to write
    }

    // Peek at front point without removing it yet
    const InputDataPoint& point = buffer_.front();

    // Convert to SMR format
    AppCmdDataType appCmd;
    appCmd.dCmdVelOffset[X_AXIS] = point.vff_x;
    appCmd.dCmdVelOffset[Y_AXIS] = point.vff_y;
    appCmd.dCmdVelOffset[Z_AXIS] = point.vff_z;
    appCmd.dDeviationCmd[X_AXIS] = point.dev_x;
    appCmd.dDeviationCmd[Y_AXIS] = point.dev_y;
    appCmd.dDeviationCmd[Z_AXIS] = point.dev_z;
    appCmd.iTargetLineNumber = getNextLineNumber();

    // Attempt write to SMR
    stats_.writeAttempts++;
    MOT_SERVICE_RETURN_CODE result = motionService_->AppWriteCmdData(&appCmd, InjectionConfig::SMR_WRITE_TIMEOUT);

    if (result == MOT_SERVICE_WRITE_SUCCESS) {
        // SUCCESS: Remove point from buffer and update stats
        buffer_.pop_front();
        stats_.totalPointsWritten++;
        return true;
    }
    else {
        // FAILURE: Keep point in buffer for retry
        stats_.writeFailures++;

        if (result == MOT_SERVICE_TIMEOUT) {
            // Expected failure - SMR buffer full
            return false;
        }
        else {
            // Unexpected error
            std::cerr << "ERROR: Unexpected SMR write error: " << result << std::endl;
            return false;
        }
    }
}

void InjectionPipeline::updateStats() {
    stats_.currentBufferSize = buffer_.size();
    stats_.bufferFillLevel = static_cast<double>(buffer_.size()) / InjectionConfig::BUFFER_CAPACITY;

    // Update status based on current conditions
    if (buffer_.empty() && !generator_->hasMoreCommands()) {
        stats_.status = "Complete";
    }
    else if (buffer_.size() < InjectionConfig::REFILL_TRIGGER / 2) {  // <25% 
        stats_.status = "Buffer Low";
    }
    else if (stats_.writeFailures > 0 && stats_.writeAttempts > 0) {
        double failureRate = static_cast<double>(stats_.writeFailures) / stats_.writeAttempts;
        if (failureRate > 0.5) {
            stats_.status = "High Write Failures";
        }
        else {
            stats_.status = "Healthy";
        }
    }
    else {
        stats_.status = "Healthy";
    }
}

void InjectionPipeline::finalFlush() {
    std::cout << "InjectionPipeline: Final flush - discarding remaining buffer..." << std::endl;

    if (!buffer_.empty()) {
        std::cout << "Discarding " << buffer_.size() << " remaining injection points" << std::endl;
        buffer_.clear();  // Just delete it - don't send to SMR
    }

    std::cout << "InjectionPipeline: Final flush complete" << std::endl;
}

void InjectionPipeline::resetLineCounter() {
    globalLineCounter_.store(0);
    countingUp_.store(true);
}

int InjectionPipeline::getNextLineNumber() {
    int current = globalLineCounter_.load();
    bool isCountingUp = countingUp_.load();

    if (isCountingUp) {
        if (current >= MAX_LINE_NUMBER) {
            countingUp_.store(false);
            return current;
        }
        else {
            globalLineCounter_.store(current + 1);
            return current;
        }
    }
    else {
        if (current <= 0) {
            countingUp_.store(true);
            return current;
        }
        else {
            globalLineCounter_.store(current - 1);
            return current;
        }
    }
}