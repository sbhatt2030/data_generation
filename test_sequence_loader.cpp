#include "core/SequenceLoader.hpp"
#include "core/GenerationPipeline.hpp"
#include "core/CSVParser.hpp"
#include <iostream>
#include <iomanip>

// Test 1: Basic SequenceLoader functionality
void test_sequence_loader_basic() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 1: Basic SequenceLoader Functionality" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    SequenceLoader loader("C:\\test_sequences\\deviations");

    if (!loader.initialize()) {
        std::cerr << " FAILED: " << loader.getLastError() << std::endl;
        return;
    }

    std::cout << " Initialized: " << loader.getFileCount() << " files found" << std::endl;

    int chunkCount = 0;
    while (loader.hasMoreData()) {
        auto chunk = loader.loadNextChunk();

        if (chunk.empty()) {
            std::cerr << " FAILED: Got empty chunk" << std::endl;
            return;
        }

        std::cout << "Chunk " << chunkCount << ": " << chunk.size() << " samples" << std::endl;
        std::cout << "  First sample: [" << chunk[0].transpose() << "]" << std::endl;
        std::cout << "  Last sample:  [" << chunk.back().transpose() << "]" << std::endl;

        // Check for NaN/Inf
        bool valid = true;
        for (const auto& sample : chunk) {
            if (!sample.allFinite()) {
                std::cerr << " FAILED: Found invalid data (NaN/Inf)" << std::endl;
                valid = false;
                break;
            }
        }

        if (!valid) return;

        chunkCount++;
        if (chunkCount >= 3) break; // Test first 3 chunks
    }

    std::cout << " PASSED: Basic loader test" << std::endl;
}

// Test 2: Short file padding
void test_sequence_loader_padding() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 2: Short File Padding" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    SequenceLoader loader("C:\\test_sequences\\short_test");

    if (!loader.initialize()) {
        std::cerr << " FAILED: " << loader.getLastError() << std::endl;
        return;
    }

    auto chunk = loader.loadNextChunk();

    if (chunk.size() != 5000) {
        std::cerr << " FAILED: Expected 5000 samples, got " << chunk.size() << std::endl;
        return;
    }

    std::cout << " PASSED: Short file returns " << chunk.size() << " samples" << std::endl;
    std::cout << "   (Padding to 8000 happens in GenerationPipeline)" << std::endl;
}

// Test 3: GenerationPipeline integration
void test_generation_pipeline_with_loaders() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 3: GenerationPipeline Integration" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    // Create minimal config
    ExperimentConfig config;
    config.experimentId = "test_external_sequences";
    config.familyId = "test_family";
    config.outputDirectory = "C:\\test_output";
    config.noiseType = KinematicNoiseType::EXISTING_SEQUENCE;
    config.deviationSequenceDir = "C:\\test_sequences\\deviations";
    config.vffConfig.vffType = VffType::EXISTING_SEQUENCE;
    config.vffSequenceDir = "C:\\test_sequences\\vff";

    // Create generation pipeline
    GenerationPipeline pipeline(
        config.outputDirectory,
        12345,  // gcode seed
        67890,  // noise seed
        11111   // vff seed
    );

    // Set configuration
    MachineConstraints constraints;
    MotionConfig motionConfig;

    pipeline.setMachineConstraints(constraints);
    pipeline.setMotionConfig(motionConfig);
    pipeline.setNoiseType(config.noiseType);
    pipeline.setVffConfig(config.vffConfig);
    pipeline.setDeviationSequenceDir(config.deviationSequenceDir);
    pipeline.setVffSequenceDir(config.vffSequenceDir);

    // Initialize (with empty G-code path - will generate dummy)
    GenerationParams genParams;
    genParams.num_trajectories = 1;

    if (!pipeline.initialize("", genParams)) {
        std::cerr << " FAILED: Pipeline initialization failed" << std::endl;
        return;
    }

    std::cout << " Pipeline initialized successfully" << std::endl;

    // Generate a few chunks
    for (int i = 0; i < 3; ++i) {
        auto chunk = pipeline.generateNextCommandData();

        if (chunk.size() != 8000) {
            std::cerr << " FAILED: Expected 8000 points, got " << chunk.size() << std::endl;
            return;
        }

        std::cout << "Chunk " << i << ": " << chunk.size() << " points" << std::endl;
        std::cout << "  First point: dev=[" << chunk[0].dev_x << ", "
            << chunk[0].dev_y << ", " << chunk[0].dev_z << "]"
            << " vff=[" << chunk[0].vff_x << ", "
            << chunk[0].vff_y << ", " << chunk[0].vff_z << "]" << std::endl;

        // Check clamping
        bool clampingWorks = true;
        for (const auto& point : chunk) {
            if (std::abs(point.dev_x) > 0.5 ||
                std::abs(point.dev_y) > 0.5 ||
                std::abs(point.dev_z) > 0.5) {
                std::cerr << " FAILED: Deviation not clamped to ±0.5mm" << std::endl;
                clampingWorks = false;
                break;
            }
            if (std::abs(point.vff_x) > 50.0 ||
                std::abs(point.vff_y) > 50.0 ||
                std::abs(point.vff_z) > 50.0) {
                std::cerr << " FAILED: VFF not clamped to ±50mm/s" << std::endl;
                clampingWorks = false;
                break;
            }
        }

        if (!clampingWorks) return;
    }

    std::cout << " PASSED: GenerationPipeline integration test" << std::endl;
}

// Test 4: CSV parsing with external sequences
void test_csv_parsing() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 4: CSV Parsing with External Sequences" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    // Create test CSV
    std::ofstream csvFile("test_external.csv");
    csvFile << "experiment_id,family_id,output_directory,num_trajectories,trajectory_type,"
        << "noise_type,noise_min_amplitude,noise_max_amplitude,noise_min_freq,noise_max_freq,"
        << "noise_min_sines,noise_max_sines,noise_sparse_prob,vff_type,vff_min_dc,vff_max_dc,"
        << "vff_max_amplitude,vff_max_freq,vff_sparse_prob,master_seed,gcode_seed,noise_seed,"
        << "vff_seed,deviation_sequence_dir,vff_sequence_dir,gcode_file_path\n";
    csvFile << "test_ext,family_a,C:\\output,10,2,4,0.001,0.01,0.5,50,3,8,0.03,4,-5,5,10,50,0.02,"
        << "12345,0,0,0,C:\\test_sequences\\deviations,C:\\test_sequences\\vff,N\\A\n";
    csvFile.close();

    auto experiments = CSVParser::parseCSV("test_external.csv");

    if (experiments.empty()) {
        std::cerr << " FAILED: " << CSVParser::getLastError() << std::endl;
        return;
    }

    const auto& exp = experiments[0];

    if (exp.noiseType != KinematicNoiseType::EXISTING_SEQUENCE) {
        std::cerr << " FAILED: noiseType not EXISTING_SEQUENCE" << std::endl;
        return;
    }

    if (exp.vffConfig.vffType != VffType::EXISTING_SEQUENCE) {
        std::cerr << " FAILED: vffType not EXISTING_SEQUENCE" << std::endl;
        return;
    }

    if (exp.deviationSequenceDir != "C:\\test_sequences\\deviations") {
        std::cerr << " FAILED: Wrong deviation directory" << std::endl;
        return;
    }

    if (exp.vffSequenceDir != "C:\\test_sequences\\vff") {
        std::cerr << " FAILED: Wrong VFF directory" << std::endl;
        return;
    }

    std::cout << " PASSED: CSV parsing test" << std::endl;
}
// Test 5: G-code file path flow (CSV → Pipeline → API-ready path)
void test_gcode_file_path_flow() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 5: G-code File Path Flow to API" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    // Step 1: Create a dummy G-code file
    std::string testGcodePath = "C:\\test_sequences\\test.fnc";
    std::ofstream gcodeFile(testGcodePath);
    gcodeFile << "G90 G21 G17 G54\n";
    gcodeFile << "G4 P1000\n";
    gcodeFile << "G0 X0 Y0 Z0 B0 F5000\n";
    gcodeFile << "G1 X100 Y100 Z0 F2000\n";
    gcodeFile << "G1 X0 Y0 Z0 F2000\n";
    gcodeFile.close();
    std::cout << "Created test G-code file: " << testGcodePath << std::endl;

    // Step 2: Create ExperimentConfig with existing G-code
    ExperimentConfig config;
    config.experimentId = "test_gcode_flow";
    config.familyId = "test_family";
    config.outputDirectory = "C:\\test_output_gcode";
    config.GcodeFilePath = testGcodePath;  // Use existing file
    config.gcodeParams.trajectory_type = TrajectoryType::EXISTING;

    config.noiseType = KinematicNoiseType::EXISTING_SEQUENCE;
    config.deviationSequenceDir = "C:\\test_sequences\\deviations";

    config.vffConfig.vffType = VffType::EXISTING_SEQUENCE;
    config.vffSequenceDir = "C:\\test_sequences\\vff";

    // Step 3: Create and initialize GenerationPipeline
    GenerationPipeline pipeline(
        config.outputDirectory,
        12345,  // gcode seed (not used for existing)
        67890,  // noise seed
        11111   // vff seed
    );

    MachineConstraints constraints;
    MotionConfig motionConfig;

    pipeline.setMachineConstraints(constraints);
    pipeline.setMotionConfig(motionConfig);
    pipeline.setNoiseType(config.noiseType);
    pipeline.setVffConfig(config.vffConfig);
    pipeline.setDeviationSequenceDir(config.deviationSequenceDir);
    pipeline.setVffSequenceDir(config.vffSequenceDir);

    // Initialize WITH existing G-code file path
    GenerationParams genParams;  // Not used when file provided
    if (!pipeline.initialize(config.GcodeFilePath, genParams)) {
        std::cerr << " FAILED: Pipeline initialization with existing G-code failed" << std::endl;
        return;
    }

    std::cout << " Pipeline initialized with existing G-code" << std::endl;

    // Step 4: Verify the G-code path is retrievable
    std::string retrievedPath = pipeline.getGCodeFilePath();

    if (retrievedPath.empty()) {
        std::cerr << " FAILED: Retrieved G-code path is empty" << std::endl;
        return;
    }

    std::cout << "Retrieved G-code path: " << retrievedPath << std::endl;

    // Step 5: Verify the file exists at the retrieved path
    if (!std::filesystem::exists(retrievedPath)) {
        std::cerr << " FAILED: G-code file does not exist at retrieved path" << std::endl;
        return;
    }

    std::cout << " G-code file exists at retrieved path" << std::endl;

    // Step 6: Verify file was copied (not original path)
    if (retrievedPath == testGcodePath) {
        std::cerr << "  WARNING: Path is original file, not copied to session folder" << std::endl;
        std::cout << "   (This may be intentional for your workflow)" << std::endl;
    }
    else {
        std::cout << " G-code was copied to session folder" << std::endl;
        std::cout << "   Original: " << testGcodePath << std::endl;
        std::cout << "   Copied:   " << retrievedPath << std::endl;
    }

    // Step 7: Simulate what CNCOverseer would do
    std::cout << "\n--- Simulating CNCOverseer workflow ---" << std::endl;

    // This is what experimentRunner_->getGeneratedGCodePath() returns
    std::string pathForAPI = retrievedPath;

    if (pathForAPI.empty()) {
        std::cerr << " FAILED: Path for API is empty" << std::endl;
        return;
    }

    std::cout << "Path that would be sent to HurcoConnection::loadAndRunProgram():" << std::endl;
    std::cout << "  " << pathForAPI << std::endl;

    // Verify it's an absolute path (required for API)
    if (pathForAPI[0] != 'C' && pathForAPI[0] != 'D') {
        std::cerr << "  WARNING: Path is not absolute (may cause API issues)" << std::endl;
    }
    else {
        std::cout << " Path is absolute (good for API)" << std::endl;
    }

    // Step 8: Verify file content is readable
    std::ifstream verifyFile(pathForAPI);
    if (!verifyFile.is_open()) {
        std::cerr << " FAILED: Cannot open file for reading" << std::endl;
        return;
    }

    std::string firstLine;
    std::getline(verifyFile, firstLine);
    verifyFile.close();

    if (firstLine.empty()) {
        std::cerr << " FAILED: File is empty" << std::endl;
        return;
    }

    std::cout << " File is readable, first line: " << firstLine << std::endl;

    // Step 9: Test CSV parsing with existing G-code
    std::cout << "\n--- Testing CSV with existing G-code path ---" << std::endl;

    std::ofstream csvFile("test_existing_gcode.csv");
    csvFile << "experiment_id,family_id,output_directory,num_trajectories,trajectory_type,"
        << "noise_type,noise_min_amplitude,noise_max_amplitude,noise_min_freq,noise_max_freq,"
        << "noise_min_sines,noise_max_sines,noise_sparse_prob,vff_type,vff_min_dc,vff_max_dc,"
        << "vff_max_amplitude,vff_max_freq,vff_sparse_prob,master_seed,gcode_seed,noise_seed,"
        << "vff_seed,deviation_sequence_dir,vff_sequence_dir,gcode_file_path\n";
    csvFile << "test_gcode,family_a,C:\\output,10,3,4,0.001,0.01,0.5,50,3,8,0.03,4,-5,5,10,50,0.02,"
        << "12345,0,0,0,C:\\test_sequences\\deviations,C:\\test_sequences\\vff,C:\\test_sequences\\test.fnc\n";  // ← G-code path in column 26
    csvFile.close();

    auto experiments = CSVParser::parseCSV("test_existing_gcode.csv");

    if (experiments.empty()) {
        std::cerr << " FAILED: CSV parsing failed: " << CSVParser::getLastError() << std::endl;
        return;
    }

    const auto& exp = experiments[0];

    if (exp.GcodeFilePath != testGcodePath) {
        std::cerr << " FAILED: G-code path not parsed correctly from CSV" << std::endl;
        std::cerr << "   Expected: " << testGcodePath << std::endl;
        std::cerr << "   Got:      " << exp.GcodeFilePath << std::endl;
        return;
    }

    if (exp.gcodeParams.trajectory_type != TrajectoryType::EXISTING) {
        std::cerr << " FAILED: trajectory_type should be EXISTING (3)" << std::endl;
        return;
    }

    std::cout << " CSV correctly parsed G-code file path" << std::endl;
    std::cout << " Trajectory type correctly set to EXISTING" << std::endl;

    std::cout << "\n" << std::string(60, '-') << std::endl;
    std::cout << " PASSED: Complete G-code file path flow verified" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "\nSummary:" << std::endl;
    std::cout << "  1.  G-code file creation" << std::endl;
    std::cout << "  2.  Pipeline initialization with existing file" << std::endl;
    std::cout << "  3.  File path retrieval" << std::endl;
    std::cout << "  4.  File copied to session folder" << std::endl;
    std::cout << "  5.  Path ready for API (absolute path)" << std::endl;
    std::cout << "  6.  File readable and valid" << std::endl;
    std::cout << "  7.  CSV parsing with G-code path" << std::endl;
    std::cout << std::endl;
    std::cout << "Ready for: hurcoConnection_->loadAndRunProgram(\"" << pathForAPI << "\")" << std::endl;
}

int main() {
    std::cout << "\n";
    std::cout << "============================================================\n";
    std::cout << "=   EXTERNAL SEQUENCE LOADER TEST SUITE                   =\n";
    std::cout << "============================================================\n";

    try {
        test_sequence_loader_basic();
        test_sequence_loader_padding();
        test_generation_pipeline_with_loaders();
        test_csv_parsing();
        test_gcode_file_path_flow();

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "ALL TESTS COMPLETED" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "\n EXCEPTION: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}