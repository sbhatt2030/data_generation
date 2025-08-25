#include "core/HurcoConnection.hpp"
#include <iostream>
#include <filesystem>
#include <regex>
#include <array>
#include <memory>

HurcoConnection::HurcoConnection()
    : wrapperRunning_(false)
    , connected_(false)
    , lastKnownStatus_(-1)
#ifdef _WIN32
    , pythonProcess_(INVALID_HANDLE_VALUE)
    , pythonStdin_(INVALID_HANDLE_VALUE)
    , pythonStdout_(INVALID_HANDLE_VALUE)
    , pythonProcessId_(0)
#else
    , pythonProcessId_(-1)
    , pythonStdin_(-1)
    , pythonStdout_(-1)
#endif
{
}

HurcoConnection::~HurcoConnection() {
    stopConnection();
}

bool HurcoConnection::startConnection() {
    if (wrapperRunning_.load()) {
        return true;
    }

    std::cout << "Starting persistent CNC connection..." << std::endl;

    // Start Python wrapper
    if (!startPythonWrapper()) {
        setError("Failed to start Python wrapper");
        return false;
    }

    wrapperRunning_ = true;

    // Start response reading thread (low priority)
    responseThread_ = std::thread(&HurcoConnection::responseThreadFunction, this);

    // Wait for initial connection response
    auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (!connected_.load() && std::chrono::steady_clock::now() < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!connected_.load()) {
        setError("Connection timeout");
        stopConnection();
        return false;
    }

    std::cout << "✅ Persistent CNC connection established" << std::endl;
    return true;
}

void HurcoConnection::stopConnection() {
    if (!wrapperRunning_.load()) {
        return;
    }

    std::cout << "Stopping persistent CNC connection..." << std::endl;

    // Send shutdown command
    sendCommand("shutdown");

    wrapperRunning_ = false;
    stopPythonWrapper();

    // Wait for response thread
    if (responseThread_.joinable()) {
        responseThread_.join();
    }

    connected_ = false;
    std::cout << "Persistent CNC connection stopped" << std::endl;
}

bool HurcoConnection::loadAndRunProgram(const std::string& gcodeFilePath) {
    if (!connected_.load()) {
        setError("Not connected to CNC");
        return false;
    }

    std::cout << "Loading and running: " << gcodeFilePath << std::endl;

    // Send command directly (blocks briefly)
    if (!sendCommand("load_and_run", gcodeFilePath)) {
        setError("Failed to send load_and_run command");
        return false;
    }

    // Python wrapper automatically starts monitoring
    lastKnownStatus_ = 1; // Assume STARTED

    std::cout << "✅ Load and run command sent" << std::endl;
    return true;
}

ProgramStatus HurcoConnection::getCurrentProgramStatus() {
    return intToProgramStatus(lastKnownStatus_.load());
}

ProgramStatus HurcoConnection::updateAndGetCurrentStatus() {
    // Send immediate status request
    sendCommand("status");

    // Return current cached status (will be updated by response thread)
    return getCurrentProgramStatus();
}

bool HurcoConnection::isProgramComplete() const {
    int status = lastKnownStatus_.load();
    return (status >= 2 && status <= 4); // Any completion state
}

bool HurcoConnection::startStatusMonitoring() {
    return sendCommand("start_monitoring");
}

bool HurcoConnection::stopStatusMonitoring() {
    return sendCommand("stop_monitoring");
}

//bool HurcoConnection::sendCommand(const std::string& command, const std::string& filePath) {
//    std::lock_guard<std::mutex> lock(commandMutex_);
//
//    if (!wrapperRunning_.load()) {
//        return false;
//    }
//
//    try {
//        // Create JSON command
//        std::string jsonCommand;
//        if (filePath.empty()) {
//            jsonCommand = R"({"command": ")" + command + R"("})";
//        }
//        else {
//            jsonCommand = R"({"command": ")" + command + R"(", "file_path": ")" + filePath + R"("})";
//        }
//        jsonCommand += "\n";
//
//        // Send to Python wrapper
//#ifdef _WIN32
//        DWORD bytesWritten;
//        if (!WriteFile(pythonStdin_, jsonCommand.c_str(), jsonCommand.length(), &bytesWritten, nullptr)) {
//            return false;
//        }
//        FlushFileBuffers(pythonStdin_);
//#else
//        if (write(pythonStdin_, jsonCommand.c_str(), jsonCommand.length()) == -1) {
//            return false;
//        }
//#endif
//
//        return true;
//
//    }
//    catch (const std::exception& e) {
//        setError("Failed to send command: " + std::string(e.what()));
//        return false;
//    }
//}

// Add this simple test version of responseThreadFunction:

void HurcoConnection::responseThreadFunction() {
    std::cout << "Response thread started - reading Python wrapper responses" << std::endl;

    while (wrapperRunning_.load()) {
        try {
            std::string response = readResponse();
            if (!response.empty()) {
                std::cout << "Python: " << response << std::flush;
                updateStatusFromResponse(response);
            }
        }
        catch (const std::exception& e) {
            std::cout << "Response thread exception: " << e.what() << std::endl;
        }

        // Brief sleep to prevent 100% CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Response thread ending" << std::endl;
}

//std::string HurcoConnection::readResponse() {
//    if (!wrapperRunning_.load()) {
//        return "";
//    }
//
//    std::string line;
//    char buffer[1024];
//
//#ifdef _WIN32
//    DWORD bytesRead;
//    if (ReadFile(pythonStdout_, buffer, sizeof(buffer) - 1, &bytesRead, nullptr)) {
//        if (bytesRead > 0) {
//            buffer[bytesRead] = '\0';
//            line = buffer;
//        }
//    }
//#else
//    ssize_t bytesRead = read(pythonStdout_, buffer, sizeof(buffer) - 1);
//    if (bytesRead > 0) {
//        buffer[bytesRead] = '\0';
//        line = buffer;
//    }
//#endif
//
//    return line;
//}

void HurcoConnection::updateStatusFromResponse(const std::string& jsonResponse) {
    try {
        // Simple JSON parsing to extract status
        size_t statusPos = jsonResponse.find("\"status_code\":");
        if (statusPos != std::string::npos) {
            size_t colonPos = jsonResponse.find(":", statusPos);
            size_t endPos = jsonResponse.find_first_of(",}", colonPos);

            if (colonPos != std::string::npos && endPos != std::string::npos) {
                std::string statusStr = jsonResponse.substr(colonPos + 1, endPos - colonPos - 1);
                statusStr.erase(std::remove_if(statusStr.begin(), statusStr.end(), ::isspace), statusStr.end());
                statusStr.erase(std::remove(statusStr.begin(), statusStr.end(), '"'), statusStr.end());

                int newStatus = std::stoi(statusStr);
                lastKnownStatus_.store(newStatus);  // Just update status, no callbacks
            }
        }

        // Check for connection status
        if (jsonResponse.find("\"status\": \"connected\"") != std::string::npos) {
            connected_ = true;
        }

    }
    catch (const std::exception& e) {
        // Silent failure for response parsing
    }
}

ProgramStatus HurcoConnection::intToProgramStatus(int statusInt) {
    switch (statusInt) {
    case 0: return ProgramStatus::UNINITIALIZED;
    case 1: return ProgramStatus::STARTED;
    case 2: return ProgramStatus::COMPLETED_SUCCESSFUL;
    case 3: return ProgramStatus::COMPLETED_ERROR;
    case 4: return ProgramStatus::COMPLETED_ABORT;
    case 5: return ProgramStatus::PENDING;
    case 6: return ProgramStatus::CHECKING_READY;
    case 7: return ProgramStatus::READY_CHECK_FAILED;
    default: return ProgramStatus::UNKNOWN;
    }
}

std::string HurcoConnection::programStatusToString(ProgramStatus status) {
    switch (status) {
    case ProgramStatus::UNINITIALIZED: return "Uninitialized (0)";
    case ProgramStatus::STARTED: return "Started (1)";
    case ProgramStatus::COMPLETED_SUCCESSFUL: return "Completed Successful (2)";
    case ProgramStatus::COMPLETED_ERROR: return "Completed Error (3)";
    case ProgramStatus::COMPLETED_ABORT: return "Completed Abort (4)";
    case ProgramStatus::PENDING: return "Pending (5)";
    case ProgramStatus::CHECKING_READY: return "Checking Ready (6)";
    case ProgramStatus::READY_CHECK_FAILED: return "Ready Check Failed (7)";
    case ProgramStatus::UNKNOWN: return "Unknown (-1)";
    default: return "Invalid Status";
    }
}

void HurcoConnection::setError(const std::string& error) {
    std::lock_guard<std::mutex> lock(errorMutex_);
    lastError_ = error;
    std::cerr << "HurcoConnection Error: " << error << std::endl;
}

// Windows-specific process management for HurcoConnection.cpp

bool HurcoConnection::startPythonWrapper() {
    try {
        // Create pipes for communication
        SECURITY_ATTRIBUTES saAttr;
        saAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
        saAttr.bInheritHandle = TRUE;
        saAttr.lpSecurityDescriptor = NULL;

        // Create pipes for stdin
        HANDLE childStdinRead, childStdinWrite;
        if (!CreatePipe(&childStdinRead, &childStdinWrite, &saAttr, 0)) {
            setError("Failed to create stdin pipe");
            return false;
        }

        // Create pipes for stdout
        HANDLE childStdoutRead, childStdoutWrite;
        if (!CreatePipe(&childStdoutRead, &childStdoutWrite, &saAttr, 0)) {
            CloseHandle(childStdinRead);
            CloseHandle(childStdinWrite);
            setError("Failed to create stdout pipe");
            return false;
        }

        // Ensure the read/write handles are not inherited
        if (!SetHandleInformation(childStdinWrite, HANDLE_FLAG_INHERIT, 0) ||
            !SetHandleInformation(childStdoutRead, HANDLE_FLAG_INHERIT, 0)) {
            CloseHandle(childStdinRead);
            CloseHandle(childStdinWrite);
            CloseHandle(childStdoutRead);
            CloseHandle(childStdoutWrite);
            setError("Failed to set handle inheritance");
            return false;
        }

        // Set up process startup info
        STARTUPINFOA startupInfo;
        ZeroMemory(&startupInfo, sizeof(startupInfo));
        startupInfo.cb = sizeof(startupInfo);
        startupInfo.hStdError = childStdoutWrite;
        startupInfo.hStdOutput = childStdoutWrite;
        startupInfo.hStdInput = childStdinRead;
        startupInfo.dwFlags |= STARTF_USESTDHANDLES;

        // Create command line
        std::string commandLine = "python\\python.exe python\\persistent_cnc_wrapper.py";

        // Create process
        PROCESS_INFORMATION processInfo;
        ZeroMemory(&processInfo, sizeof(processInfo));

        BOOL success = CreateProcessA(
            NULL,                           // No module name (use command line)
            const_cast<char*>(commandLine.c_str()), // Command line
            NULL,                           // Process handle not inheritable
            NULL,                           // Thread handle not inheritable
            TRUE,                           // Set handle inheritance to TRUE
            0,                              // No creation flags
            NULL,                           // Use parent's environment block
            NULL,                           // Use parent's starting directory
            &startupInfo,                   // Pointer to STARTUPINFO structure
            &processInfo                    // Pointer to PROCESS_INFORMATION structure
        );

        // Close handles that child inherited
        CloseHandle(childStdinRead);
        CloseHandle(childStdoutWrite);

        if (!success) {
            CloseHandle(childStdinWrite);
            CloseHandle(childStdoutRead);
            setError("Failed to create Python process");
            return false;
        }

        // Store process information
        pythonProcess_ = processInfo.hProcess;
        pythonProcessId_ = processInfo.dwProcessId;
        pythonStdin_ = childStdinWrite;
        pythonStdout_ = childStdoutRead;

        // Close thread handle (we don't need it)
        CloseHandle(processInfo.hThread);

        std::cout << "Python wrapper process started (PID: " << pythonProcessId_ << ")" << std::endl;
        return true;

    }
    catch (const std::exception& e) {
        setError("Exception starting Python wrapper: " + std::string(e.what()));
        return false;
    }
}

void HurcoConnection::stopPythonWrapper() {
    if (pythonProcess_ == INVALID_HANDLE_VALUE) {
        return;
    }

    std::cout << "Stopping Python wrapper process..." << std::endl;

    // Close communication pipes first
    if (pythonStdin_ != INVALID_HANDLE_VALUE) {
        CloseHandle(pythonStdin_);
        pythonStdin_ = INVALID_HANDLE_VALUE;
    }

    if (pythonStdout_ != INVALID_HANDLE_VALUE) {
        CloseHandle(pythonStdout_);
        pythonStdout_ = INVALID_HANDLE_VALUE;
    }

    // Wait for process to exit gracefully
    DWORD waitResult = WaitForSingleObject(pythonProcess_, 5000); // 5 second timeout

    if (waitResult == WAIT_TIMEOUT) {
        std::cout << "Python process didn't exit gracefully, terminating..." << std::endl;
        TerminateProcess(pythonProcess_, 1);
        WaitForSingleObject(pythonProcess_, 2000); // Wait up to 2 more seconds
    }

    // Close process handle
    CloseHandle(pythonProcess_);
    pythonProcess_ = INVALID_HANDLE_VALUE;
    pythonProcessId_ = 0;

    std::cout << "Python wrapper process stopped" << std::endl;
}

std::string HurcoConnection::readResponse() {
    if (!wrapperRunning_.load() || pythonStdout_ == INVALID_HANDLE_VALUE) {
        return "";
    }

    char buffer[4096];
    DWORD bytesRead = 0;

    // Set a reasonable timeout for reading
    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = 100;         // 100ms between characters
    timeouts.ReadTotalTimeoutConstant = 1000;   // 1 second total timeout
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    // Note: SetCommTimeouts only works for serial ports, not pipes
    // For pipes, ReadFile will block until data is available or pipe is closed

    if (ReadFile(pythonStdout_, buffer, sizeof(buffer) - 1, &bytesRead, NULL)) {
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            return std::string(buffer);
        }
    }
    else {
        DWORD error = GetLastError();
        if (error == ERROR_BROKEN_PIPE) {
            // Python process ended
            wrapperRunning_ = false;
        }
    }

    return "";
}

bool HurcoConnection::sendCommand(const std::string& command, const std::string& filePath) {
    std::lock_guard<std::mutex> lock(commandMutex_);

    if (!wrapperRunning_.load() || pythonStdin_ == INVALID_HANDLE_VALUE) {
        return false;
    }

    try {
        // Create JSON command
        std::string jsonCommand;
        if (filePath.empty()) {
            jsonCommand = R"({"command": ")" + command + R"("})";
        }
        else {
            // Escape backslashes in file path for JSON
            std::string escapedPath = filePath;
            size_t pos = 0;
            while ((pos = escapedPath.find("\\", pos)) != std::string::npos) {
                escapedPath.replace(pos, 1, "\\\\");
                pos += 2;
            }
            jsonCommand = R"({"command": ")" + command + R"(", "file_path": ")" + escapedPath + R"("})";
        }
        jsonCommand += "\n";

        // Send to Python wrapper
        DWORD bytesWritten = 0;
        if (!WriteFile(pythonStdin_, jsonCommand.c_str(), static_cast<DWORD>(jsonCommand.length()), &bytesWritten, NULL)) {
            DWORD error = GetLastError();
            setError("Failed to write to Python process (error: " + std::to_string(error) + ")");
            return false;
        }

        // Ensure data is sent immediately
        FlushFileBuffers(pythonStdin_);
        return true;

    }
    catch (const std::exception& e) {
        setError("Failed to send command: " + std::string(e.what()));
        return false;
    }
}

void HurcoConnection::resetStatusForNewExperiment() {
    std::cout << "🔄 Resetting CNC status for new experiment..." << std::endl;


    // Step 1: Tell Python wrapper to reset its status
    std::cout << "Step 1: Telling Python wrapper to reset status..." << std::endl;
    if (!sendCommand("reset_status")) {
        std::cerr << "WARNING: Failed to send reset_status command to Python wrapper" << std::endl;
    }

    // Step 2: Reset our local status cache
    int oldStatus = lastKnownStatus_.exchange(-1);  // Set to UNKNOWN
    std::cout << "Step 2: Reset local status from "
        << programStatusToString(intToProgramStatus(oldStatus))
        << " to " << programStatusToString(ProgramStatus::UNKNOWN) << std::endl;

    // Step 3: Give time for Python wrapper to process reset
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Step 4: Force fresh status reading
    std::cout << "Step 3: Requesting fresh status reading..." << std::endl;
    sendCommand("status");

    // Step 5: Wait for status response to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    ProgramStatus finalStatus = getCurrentProgramStatus();
    std::cout << "✅ Status reset complete. Current status: "
        << programStatusToString(finalStatus) << std::endl;

    // If status is still a completion status, warn about potential issue
    if (finalStatus == ProgramStatus::COMPLETED_SUCCESSFUL ||
        finalStatus == ProgramStatus::COMPLETED_ERROR ||
        finalStatus == ProgramStatus::COMPLETED_ABORT) {
        std::cout << "⚠️  WARNING: Status still shows completion after reset. "
            << "CNC may need manual intervention." << std::endl;
    }
}
