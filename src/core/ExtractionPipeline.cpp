#include "core/ExtractionPipeline.hpp"
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <fstream>


ExtractionPipeline::ExtractionPipeline()
    : motionService_(nullptr)
    , bufferHasMotion_(false)
    , currentLineNumber_(-1)
    , lastLogPoint_(0)
    , fileCounter_(1) {

    // Pre-allocate buffer for performance
    currentBuffer_.reserve(ExtractionConfig::FIXED_BUFFER_SIZE);
}

bool ExtractionPipeline::initialize(MotionService* motionService,
    const std::string& sessionFolder,
    const std::string& spikeLogPath) {
    fileCounter_ = 1;

    if (!motionService) {
        std::cerr << "ERROR: MotionService pointer is null" << std::endl;
        return false;
    }

    if (sessionFolder.empty()) {
        std::cerr << "ERROR: Session folder is empty" << std::endl;
        return false;
    }

    motionService_ = motionService;
    sessionFolder_ = sessionFolder;
    resultsFolder_ = sessionFolder_ + "\\results";

    // Create results directory
    if (!std::filesystem::exists(resultsFolder_)) {
        try {
            std::filesystem::create_directories(resultsFolder_);
        }
        catch (const std::exception& e) {
            std::cerr << "ERROR: Failed to create results folder: " << e.what() << std::endl;
            return false;
        }
    }

    // Initialize statistics
    stats_ = ExtractionStats{};
    stats_.status = "Initialized";
    lastLogPoint_ = 0;

    // Store spike log path for later use
    spikeLogPath_ = spikeLogPath;

    // Reset spike detection state
    hasPreviousPosition_ = false;
    positionSpikeCount_ = 0;

    // Create spike log file with header if path provided
    if (!spikeLogPath_.empty()) {
        try {
            // Ensure directory exists
            std::filesystem::create_directories(std::filesystem::path(spikeLogPath_).parent_path());

            // Write header to spike log file (only if file doesn't exist)
            if (!std::filesystem::exists(spikeLogPath_)) {
                std::ofstream spikeLog(spikeLogPath_, std::ios::app);
                if (spikeLog.is_open()) {
                    spikeLog << "# Position Spike Log - Threshold: 1.0mm\n";
                    spikeLog << "# Format: Timestamp - SPIKE #N: ΔX=Xmm, ΔY=Ymm, ΔZ=Zmm (Line: N, Experiment: ID)\n";
                    spikeLog << "# Session: " << sessionFolder << "\n";
                    spikeLog << "# Started: " << getCurrentTimestamp() << "\n";
                    spikeLog << "#" << std::string(80, '=') << "\n";
                    spikeLog.close();
                    std::cout << "Position spike logging enabled: " << spikeLogPath_ << std::endl;
                }
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Warning: Could not initialize spike log: " << e.what() << std::endl;
            spikeLogPath_.clear(); // Disable spike logging on error
        }
    }

    // Reset collection state
    resetBuffer();

    std::cout << "Sequential Extraction Pipeline initialized successfully" << std::endl;
    std::cout << "File counter reset to 1 for new experiment" << std::endl;
    std::cout << "  Results folder: " << resultsFolder_ << std::endl;
    std::cout << "  Fixed buffer size: " << ExtractionConfig::FIXED_BUFFER_SIZE << " points" << std::endl;

    return true;
}

void ExtractionPipeline::processOneCycle() {
    // 1. Read and process ALL available points from SMR output buffer
    readAllAvailablePoints();

    // 2. Check if buffer should be dumped (full = 8000 points)
    if (shouldDumpBuffer()) {
        dumpBufferToFile();
    }

    // 3. Update internal statistics
    updateStats();

    // 4. Check if extraction is complete
    checkExtractionCompletion();

    // 5. Periodic logging
    if (stats_.totalPointsRead - lastLogPoint_ >= ExtractionConfig::LOGGING_INTERVAL) {
        logStatus();
        lastLogPoint_ = stats_.totalPointsRead;
    }
}

bool ExtractionPipeline::shouldContinueExtracting() const {

    // Priority 2: Existing completion logic
    if (stats_.extractionComplete) {
        return false;
    }

    // Continue extracting if neither condition is met
    return true;
}


void ExtractionPipeline::finalFlush() {
    std::cout << "ExtractionPipeline: Final flush..." << std::endl;

    // Just dump current buffer if it has motion data - no more SMR reading
    if (!currentBuffer_.empty() && bufferHasMotion_) {
        std::cout << "Final flush: dumping remaining buffer with motion data ("
            << currentBuffer_.size() << " points)" << std::endl;
        dumpBufferToFile();
    }
    else if (!currentBuffer_.empty()) {
        std::cout << "Final flush: discarding buffer with no motion data ("
            << currentBuffer_.size() << " points)" << std::endl;
        resetBuffer();  // Clear it without saving
    }

    std::cout << "ExtractionPipeline: Final flush complete" << std::endl;
}

ExtractionStats ExtractionPipeline::getStats() const {
    // Update current stats before returning
    const_cast<ExtractionPipeline*>(this)->updateStats();
    return stats_;
}

void ExtractionPipeline::logStatus() const {
    std::cout << "=== Sequential Extraction Status ===" << std::endl;
    std::cout << "Points Read: " << stats_.totalPointsRead << std::endl;
    std::cout << "Position Spikes: " << stats_.positionSpikeCount << std::endl;
    std::cout << "Current Buffer: " << currentBuffer_.size() << "/"
        << ExtractionConfig::FIXED_BUFFER_SIZE;

    if (bufferHasMotion_) {
        std::cout << " (HAS MOTION)";
    }
    else {
        std::cout << " (no motion)";
    }
    std::cout << std::endl;

    std::cout << "Current Line: " << currentLineNumber_<< std::endl;
    std::cout << "Files Written: " << stats_.csvFilesWritten << std::endl;

    double readSuccessRate = (stats_.readAttempts > 0) ?
        (100.0 * (stats_.readAttempts - stats_.readFailures) / stats_.readAttempts) : 0.0;
    std::cout << "Read Success Rate: " << std::fixed << std::setprecision(1)
        << readSuccessRate << "%" << std::endl;

    std::cout << "Status: " << stats_.status << std::endl;
    std::cout << "===================================" << std::endl;
}

// Private methods implementation

void ExtractionPipeline::readAllAvailablePoints() {
    stats_.status = "Reading Data";

    // Try to drain SMR buffer until our buffer is full or no more data available
    while (currentBuffer_.size() < ExtractionConfig::FIXED_BUFFER_SIZE) {
        ExtractedDataPoint point;

        if (readSinglePoint(point)) {
            addPointToBuffer(point);
        }
        else {
            // No more data available in SMR buffer - break and try again next cycle
            break;
        }
    }
}

bool ExtractionPipeline::readSinglePoint(ExtractedDataPoint& point) {
    RTMotionDataType rtData;

    stats_.readAttempts++;
    MOT_SERVICE_RETURN_CODE result = motionService_->AppReadMotionData(&rtData, ExtractionConfig::SMR_READ_TIMEOUT);

    if (result == MOT_SERVICE_READ_SUCCESS) {
        // ⭐ SPIKE DETECTION RIGHT HERE - before any conversion
        if (hasPreviousPosition_) {
            double deltaX = std::abs(rtData.dPosition[X_AXIS] - prevPosX_);
            double deltaY = std::abs(rtData.dPosition[Y_AXIS] - prevPosY_);
            double deltaZ = std::abs(rtData.dPosition[Z_AXIS] - prevPosZ_);

            if (deltaX > 1.0 || deltaY > 1.0 || deltaZ > 1.0) {
                std::cout << "SPIKE AT SMR READ: ΔX=" << deltaX
                    << " ΔY=" << deltaY << " ΔZ=" << deltaZ
                    << " line=" << rtData.iCurrentLineNumber << std::endl;

                // Log raw SMR data
                std::cout << "Raw SMR data: pos=[" << rtData.dPosition[X_AXIS]
                    << "," << rtData.dPosition[Y_AXIS]
                    << "," << rtData.dPosition[Z_AXIS] << "]" << std::endl;
				logPositionSpike(deltaX, deltaY, deltaZ, rtData.iCurrentLineNumber);
            }
        }

        // Store for next comparison
        prevPosX_ = rtData.dPosition[X_AXIS];
        prevPosY_ = rtData.dPosition[Y_AXIS];
        prevPosZ_ = rtData.dPosition[Z_AXIS];
		hasPreviousPosition_ = true;

        // Convert RTMotionDataType to ExtractedDataPoint
        point.pos_x = rtData.dPosition[X_AXIS];
        point.pos_y = rtData.dPosition[Y_AXIS];
        point.pos_z = rtData.dPosition[Z_AXIS];
        point.dev_x = rtData.dDeviation[X_AXIS];
        point.dev_y = rtData.dDeviation[Y_AXIS];
        point.dev_z = rtData.dDeviation[Z_AXIS];
        point.vff_x = rtData.dVffApplied[X_AXIS];
        point.vff_y = rtData.dVffApplied[Y_AXIS];
        point.vff_z = rtData.dVffApplied[Z_AXIS];
        point.e_enc_x = rtData.dEncoderError[X_AXIS];
        point.e_enc_y = rtData.dEncoderError[Y_AXIS];
        point.e_enc_z = rtData.dEncoderError[Z_AXIS];
        point.e_scale_x = rtData.dScaleError[X_AXIS];
        point.e_scale_y = rtData.dScaleError[Y_AXIS];
        point.e_scale_z = rtData.dScaleError[Z_AXIS];
        point.line_number = rtData.iCurrentLineNumber;
        point.has_motion = rtData.bHasMotion;

        stats_.totalPointsRead++;
        if (point.has_motion) {
            stats_.pointsWithMotion++;  // Simple increment
        }
        return true;
    }
    else {
        stats_.readFailures++;
        return false;
    }
}

void ExtractionPipeline::addPointToBuffer(const ExtractedDataPoint& point) {
    // Add every point to buffer (both motion and no-motion)
    currentBuffer_.push_back(point);
    stats_.totalPointsKept++;

    // Update motion tracking
    if (point.has_motion) {
        bufferHasMotion_ = true;
        stats_.motionPointsInBuffer++;
    }
}


bool ExtractionPipeline::shouldDumpBuffer() const {
    // Dump when buffer reaches exactly 8000 points
    return currentBuffer_.size() >= ExtractionConfig::FIXED_BUFFER_SIZE;
}

void ExtractionPipeline::dumpBufferToFile() {
    if (currentBuffer_.empty()) {
        resetBuffer();
        return;
    }

    // Only dump buffers that contain motion data
    if (!bufferHasMotion_) {
        std::cout << "Discarding buffer: no motion data (size: " << currentBuffer_.size() << ")" << std::endl;
        resetBuffer();
        return;
    }
    std::string filename = generateSequentialFilename();

    std::cout << "Dumping buffer to " << filename
        << " (size: " << currentBuffer_.size()
        << ", motion points: " << stats_.motionPointsInBuffer << ")" << std::endl;

    if (writeBufferToCSV(filename)) {
        stats_.csvFilesWritten++;
        std::cout << "Successfully wrote file " << stats_.csvFilesWritten
            << ": " << filename << std::endl;
    }
    else {
        stats_.csvWriteErrors++;
        std::cerr << "ERROR: Failed to write file: " << filename << std::endl;
    }

    resetBuffer();
}

std::string ExtractionPipeline::generateSequentialFilename() {
    std::string filename = resultsFolder_ + "\\data_"
        + std::to_string(fileCounter_++)
        + ".csv";
    return filename;
}

bool ExtractionPipeline::writeBufferToCSV(const std::string& filename) {
    try {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "ERROR: Failed to open file: " << filename << std::endl;
            return false;
        }

        // Write header
        file << "pos_x,pos_y,pos_z,"
            << "dev_x,dev_y,dev_z,"
            << "vff_x,vff_y,vff_z,"
            << "e_enc_x,e_enc_y,e_enc_z,"
            << "e_scale_x,e_scale_y,e_scale_z,"
            << "line_number,has_motion\n";

        // Write data with scientific notation for precision
        file << std::scientific << std::setprecision(6);

        for (const auto& point : currentBuffer_) {
            file << point.pos_x << "," << point.pos_y << "," << point.pos_z << ","
                << point.dev_x << "," << point.dev_y << "," << point.dev_z << ","
                << point.vff_x << "," << point.vff_y << "," << point.vff_z << ","
                << point.e_enc_x << "," << point.e_enc_y << "," << point.e_enc_z << ","
                << point.e_scale_x << "," << point.e_scale_y << "," << point.e_scale_z << ","
                << point.line_number << "," << (point.has_motion ? 1 : 0) << "\n";
        }

        file.close();
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Exception writing CSV: " << e.what() << std::endl;
        return false;
    }
}

void ExtractionPipeline::resetBuffer() {
    currentBuffer_.clear();
    bufferHasMotion_ = false;
    stats_.motionPointsInBuffer = 0;
}

void ExtractionPipeline::checkExtractionCompletion(){ 
}

    // If external control is enabled, let external condition handle completion
    //if (externalStopCondition_) {
    //    // Only set completion if external condition says to stop AND we have no buffered motion data
    //    if (*externalStopCondition_) {
    //        // IMMEDIATE COMPLETION when CNC says it's done - ignore buffer state
    //        stats_.extractionComplete = true;
    //        stats_.status = "Complete (CNC Status 2/3/4)";
    //        std::cout << "*** EXTRACTION COMPLETE (CNC COMPLETION DETECTED) ***" << std::endl;
    //        std::cout << "  - CNC program completed (status 2/3/4)" << std::endl;
    //        std::cout << "  - CSV files written: " << stats_.csvFilesWritten << std::endl;
    //        std::cout << "  - Current buffer: " << currentBuffer_.size() << " points" << std::endl;

    //        // Save any remaining motion data
    //        if (bufferHasMotion_ && !currentBuffer_.empty()) {
    //            std::cout << "  - Final buffer dump: saving " << currentBuffer_.size() << " points" << std::endl;
    //            dumpBufferToFile();
    //        }
    //    }
    //    return;  // Skip line-number based logic when external control is active
    //}

    // Original line-number based completion logic (fallback when no external control)
 

void ExtractionPipeline::updateStats() {
    stats_.currentBufferSize = currentBuffer_.size();
    stats_.currentLineNumber = currentLineNumber_;
    stats_.positionSpikeCount = positionSpikeCount_;

    // Update status based on current conditions
    if (stats_.extractionComplete) {
        stats_.status = "Exctraction Complete";
    }
    else if (stats_.readFailures > 0 && stats_.readAttempts > 0) {
        double failureRate = static_cast<double>(stats_.readFailures) / stats_.readAttempts;
        if (failureRate > 0.5) {
            stats_.status = "High Read Failures";
        }
        else {
            stats_.status = "Healthy";
        }
    }
    else {
        stats_.status = "Healthy";
    }
}
void ExtractionPipeline::logPositionSpike(double deltaX, double deltaY, double deltaZ, int lineNumber) {
    // Create spike message
    std::ostringstream spikeMsg;
    spikeMsg << "POSITION SPIKE #" << positionSpikeCount_ << ": "
        << "ΔX=" << std::fixed << std::setprecision(3) << deltaX << "mm, "
        << "ΔY=" << deltaY << "mm, "
        << "ΔZ=" << deltaZ << "mm "
        << "(Line: " << lineNumber << ")";

    std::string timestamp = getCurrentTimestamp();

    // Log to console with warning symbol
    std::cout << "⚠️ [" << timestamp << "] " << spikeMsg.str() << std::endl;

    // Log to file if spike logging is enabled
    if (!spikeLogPath_.empty()) {
        try {
            std::ofstream spikeLog(spikeLogPath_, std::ios::app);
            if (spikeLog.is_open()) {
                spikeLog << timestamp << " - " << spikeMsg.str()
                    << " (Buffer: " << currentBuffer_.size() << " points)" << std::endl;
                spikeLog.close(); // Close immediately to ensure write
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error writing to spike log: " << e.what() << std::endl;
        }
    }
}

std::string ExtractionPipeline::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    std::tm local_time;
    localtime_s(&local_time, &time_t);
    ss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}
