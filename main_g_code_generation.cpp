#include "core/gcode_generator.hpp"
#include "core/system_constants.hpp"
#include <iostream>
#include <string>
#include <filesystem>

int main() {
    // Output directory
    std::string output_dir = "generated_gcode";
    std::filesystem::create_directories(output_dir);

    // Set up machine constraints from your system constants
    MachineConstraints constraints;
    constraints.min_position = SystemConstants::Machine::WORKSPACE_MIN;
    constraints.max_position = SystemConstants::Machine::WORKSPACE_MAX;
    constraints.max_velocity = SystemConstants::Machine::MAX_VELOCITY;
    constraints.max_acceleration = SystemConstants::Machine::MAX_ACCELERATION;
    constraints.max_jerk = SystemConstants::Machine::MAX_JERK;
    constraints.max_feedrate = SystemConstants::Machine::MAX_FEEDRATE_MM_PER_MIN;

    // G-code generation parameters (mixed trajectories)
    GenerationParams params;
    params.num_trajectories = 1000;                    // Adjust as needed
    params.trajectory_type = TrajectoryType::MIXED;
    params.linear_probability = 0.5;                 // 60% linear, 40% circular
    params.max_trajectory_time = 2.0;
    params.dwell_time = 0.05;
    params.use_dwell_commands = true;
    params.write_summary_to_file = true;

    // How many files do you want?
    int num_files = 40;

    for (int i = 1; i <= num_files; ++i) {
        std::cout << "Generating G-code file " << i << "/" << num_files << "..." << std::endl;

        // Create generator with random seed (or use i for deterministic)
        GCodeGenerator generator(constraints, i);  // Use i as seed for variety

        // Generate filename
        std::string filename = output_dir + "/program_" + std::to_string(i) + ".fnc";

        // Set summary output directory
        params.summary_output_directory = output_dir;

        // Generate the file
        if (generator.generateGCodeFile(filename, params)) {
            std::cout << "✅ Generated: " << filename << std::endl;
        }
        else {
            std::cout << "❌ Failed to generate: " << filename << std::endl;
        }
    }

    std::cout << "\nDone! Generated " << num_files << " G-code files in: " << output_dir << std::endl;

    return 0;
}