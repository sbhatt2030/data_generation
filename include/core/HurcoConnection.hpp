// Simplified HurcoConnection.hpp - No queuing, direct commands

#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#else
#include <unistd.h>
#include <sys/wait.h>
#endif
#include <functional>

enum class ProgramStatus {
    UNINITIALIZED = 0,
    STARTED = 1,
    COMPLETED_SUCCESSFUL = 2,
    COMPLETED_ERROR = 3,
    COMPLETED_ABORT = 4,
    PENDING = 5,
    CHECKING_READY = 6,
    READY_CHECK_FAILED = 7,
    UNKNOWN = -1
};

/**
 * Simplified HurcoConnection - No queuing, direct communication
 */
class HurcoConnection {
public:
    HurcoConnection();
    ~HurcoConnection();

    /**
     * Start persistent connection (connects to CNC automatically)
     */
    bool startConnection();

    /**
     * Stop persistent connection
     */
    void stopConnection();

    /**
     * Check if connected
     */
    bool isConnected() const { return connected_.load(); }

    /**
     * Load and run program (blocks briefly to send command)
     * Automatically starts background status monitoring
     */
    bool loadAndRunProgram(const std::string& gcodeFilePath);

    /**
     * Get current status (from cached value updated by background thread)
     */
    ProgramStatus getCurrentProgramStatus();

    /**
     * Force immediate status update (blocks briefly)
     */
    ProgramStatus updateAndGetCurrentStatus();

    /**
     * Convenience methods
     */
    bool isProgramRunning() const;
    bool isProgramComplete() const;

    /**
     * Manual monitoring control (optional)
     */
    bool startStatusMonitoring();
    bool stopStatusMonitoring();

    /**
     * Error handling
     */
    const std::string& getLastError() const { return lastError_; }
    std::string programStatusToString(ProgramStatus status);

    /**
     * Reset CNC status for new experiment
     * Call this before starting a new experiment to clear previous status
     */
    void resetStatusForNewExperiment();

private:
    // Process management
#ifdef _WIN32
    HANDLE pythonProcess_;
    HANDLE pythonStdin_;
    HANDLE pythonStdout_;
    DWORD pythonProcessId_;
#else
    pid_t pythonProcessId_;
    int pythonStdin_;
    int pythonStdout_;
#endif

    // Simple state
    std::atomic<bool> wrapperRunning_;
    std::atomic<bool> connected_;
    std::atomic<int> lastKnownStatus_;

    // Communication thread (just reads responses)
    std::thread responseThread_;

    // Thread-safe command sending
    std::mutex commandMutex_;

    // Error handling
    mutable std::string lastError_;
    mutable std::mutex errorMutex_;
    std::function<void(ProgramStatus, ProgramStatus)> statusChangeCallback_;
    mutable std::mutex callbackMutex_;
    /**
     * Start Python wrapper process
     */
    bool startPythonWrapper();

    /**
     * Stop Python wrapper process
     */
    void stopPythonWrapper();

    /**
     * Response reading thread (low priority)
     */
    void responseThreadFunction();

    /**
     * Send command directly (blocks briefly)
     */
    bool sendCommand(const std::string& command, const std::string& filePath = "");

    /**
     * Read single response from Python
     */
    std::string readResponse();

    /**
     * Update status from response data
     */
    void updateStatusFromResponse(const std::string& jsonResponse);

    /**
     * Utility methods
     */
    ProgramStatus intToProgramStatus(int statusInt);
    void setError(const std::string& error);
};

