#include "core/CNCOverseer.hpp"
#include <iostream>
#include <string>
#include <cstdlib>
/**
 * Print usage information
 */
void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [csv_file] [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  [csv_file]              Path to CSV experiment configuration file (default: experiments.csv)" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --config <path>         System configuration file (default: config/system_config.json)" << std::endl;
    std::cout << "  --output <path>         Output directory for all experiments (default: ./batch_output)" << std::endl;
    std::cout << "  --help                  Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << programName << "                              # Uses default experiments.csv" << std::endl;
    std::cout << "  " << programName << " custom_experiments.csv       # Uses specified CSV file" << std::endl;
    std::cout << "  " << programName << " --output ./lstm_data         # Default CSV, custom output" << std::endl;
    std::cout << "  " << programName << " lstm.csv --config ./my_config.json" << std::endl;
}

/**
 * Parse command line arguments
 */
struct Arguments {
    std::string csvFile = "test.csv";  // Default CSV file
    std::string configFile = "config\\system_config.json";
    std::string outputDir = "C:\\data_collection";
    bool showHelp = false;
    bool valid = true;
    std::string errorMessage;
};

Arguments parseArguments(int argc, char* argv[]) {
    Arguments args;

    // Parse arguments - CSV file is now optional
    int argIndex = 1;

    // Check if first argument is a CSV file (doesn't start with --)
    if (argc > 1 && std::string(argv[1])[0] != '-') {
        args.csvFile = argv[1];
        argIndex = 2;
    }

    // Parse optional arguments
    for (int i = argIndex; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            args.showHelp = true;
        }
        else if (arg == "--config") {
            if (i + 1 < argc) {
                args.configFile = argv[++i];
            }
            else {
                args.valid = false;
                args.errorMessage = "--config requires a file path";
                return args;
            }
        }
        else if (arg == "--output") {
            if (i + 1 < argc) {
                args.outputDir = argv[++i];
            }
            else {
                args.valid = false;
                args.errorMessage = "--output requires a directory path";
                return args;
            }
        }
        else {
            args.valid = false;
            args.errorMessage = "Unknown argument: " + arg;
            return args;
        }
    }

    return args;
}

/**
 * Main entry point for CNC Experiment Batch Processing
 */
int main(int argc, char* argv[]) {

    std::cout << "CNC LSTM Training Data Generation System" << std::endl;
    std::cout << "========================================" << std::endl;

    // Parse command line arguments
    Arguments args = parseArguments(argc, argv);

    if (args.showHelp) {
        printUsage(argv[0]);
        return 0;
    }

    if (!args.valid) {
        std::cerr << "Error: " << args.errorMessage << std::endl;
        std::cout << std::endl;
        printUsage(argv[0]);
        return 1;
    }
    system("pause");
    // Display configuration
    std::cout << "Configuration:" << std::endl;
    std::cout << "  CSV file: " << args.csvFile << std::endl;
    std::cout << "  System config: " << args.configFile << std::endl;
    std::cout << "  Output directory: " << args.outputDir << std::endl;
    std::cout << std::endl;

    try {
        // Create and configure Overseer
        CNCOverseer overseer;

        std::cout << "Step 1: Loading system configuration..." << std::endl;
        if (!overseer.loadSystemConfiguration(args.configFile)) {
            std::cerr << "❌ Failed to load system configuration: " << overseer.getLastError() << std::endl;
            return 1;
        }
        std::cout << "✅ System configuration loaded successfully" << std::endl;

        std::cout << "\nStep 2: Connecting to CNC machine..." << std::endl;
        if (!overseer.connectToCNC()) {
            std::cerr << "❌ Failed to connect to CNC: " << overseer.getLastError() << std::endl;
            overseer.disconnectFromCNC();
            return 1;
        }
        std::cout << "✅ CNC connection established" << std::endl;

        std::cout << "\nStep 3: Starting experiment batch execution..." << std::endl;
        BatchResult result = overseer.runExperimentBatch(args.csvFile, args.outputDir);

        std::cout << "\nStep 4: Disconnecting from CNC..." << std::endl;
        overseer.disconnectFromCNC();

        // Final results
        std::cout << "\n🎯 BATCH EXECUTION COMPLETE!" << std::endl;
        std::cout << "Success rate: " << result.successfulExperiments << "/" << result.totalExperiments;

        if (result.successfulExperiments == result.totalExperiments) {
            std::cout << " (100% SUCCESS! 🎉)" << std::endl;
            return 0;
        }
        else {
            std::cout << " (" << result.failedExperiments << " failed)" << std::endl;
            std::cout << "Check error log: " << result.errorLogPath << std::endl;
            return (result.failedExperiments == result.totalExperiments) ? 1 : 0; // 0 if partial success
        }

    }
    catch (const std::exception& e) {
        std::cerr << "❌ Unexpected error: " << e.what() << std::endl;
        return 1;
        system("pause");
    }
    system("pause");
}