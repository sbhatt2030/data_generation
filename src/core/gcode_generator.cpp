#define _USE_MATH_DEFINES
#include "core/gcode_generator.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <filesystem>

// =============================================================================
// Constructor and Basic Setup
// =============================================================================
GCodeGenerator::GCodeGenerator(const MachineConstraints& constraints, unsigned int seed)
    : constraints_(constraints), current_seed_(seed) {
    if (seed == 0) {
        current_seed_ = static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count());
    }
    rng_.seed(current_seed_);
    resetPositionTracking();
}

// =============================================================================
// Main G-code File Generation
// =============================================================================
bool GCodeGenerator::generateGCodeFile(const std::string& filename,
    const GenerationParams& params) const {

    if (!params.isValid()) {
        std::cerr << "Invalid generation parameters" << std::endl;
        return false;
    }

    // Reset position tracking for new file generation
    resetPositionTracking();

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    try {
        // Write header
        file << formatHeader() << std::endl;

        // Add automatic initial dwell (10 seconds)
        file << "G4 P1000" << std::endl;
        file << "G0.0 X0.0 Y0.0 Z0.0 B0.0 F10000" << std::endl;

        // Generate trajectories based on type
        std::vector<std::string> trajectories;
        switch (params.trajectory_type) {
        case TrajectoryType::LINEAR_ONLY:
            trajectories = generateLinearTrajectories(params.num_trajectories, params);
            break;
        case TrajectoryType::CIRCULAR_ONLY:
            trajectories = generateCircularTrajectories(params.num_trajectories, params);
            break;
        case TrajectoryType::MIXED:
            trajectories = generateMixedTrajectories(params.num_trajectories, params);
            break;
        }

        // Write trajectories to file
        for (const auto& trajectory : trajectories) {
            file << trajectory << std::endl;
        }

        // ALWAYS end with return to origin
        if (params.use_dwell_commands) {
            file << formatDwellCommand(params.dwell_time) << std::endl;
        }
        file << "G1 X0.0 Y0.0 Z0.0 B90.0 F10000" << std::endl;
        file << formatDwellCommand(params.dwell_time) << std::endl;

        file.close();

        // Write summary if requested
        if (params.write_summary_to_file) {
            std::string summary = getGenerationSummary(params);
            std::string summary_filepath = generateSummaryFilename(params.summary_output_directory);

            std::ostringstream file_summary;
            file_summary << "=== G-Code Generation Summary ===" << std::endl;
            file_summary << "G-Code file: " << filename << std::endl;
            file_summary << "Summary file: " << summary_filepath << std::endl;
            file_summary << "Random seed: " << current_seed_ << std::endl;
            file_summary << "Number of trajectories generated: " << trajectories.size() << std::endl;
            file_summary << "Initial dwell: 10 seconds (automatic)" << std::endl;
            file_summary << "Final move: Return to origin (0,0,0)" << std::endl;
            file_summary << "Continuous motion: YES (no G0 moves)" << std::endl;
            file_summary << std::endl;
            file_summary << summary;

            writeSummaryToFile(file_summary.str(), summary_filepath);
            std::cout << "Summary written to: " << summary_filepath << std::endl;
        }

        std::cout << "G-code generation complete with " << trajectories.size()
            << " trajectories plus return to origin" << std::endl;

        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error generating G-code file: " << e.what() << std::endl;
        return false;
    }
}

// =============================================================================
// Trajectory Generation Methods
// =============================================================================
std::vector<std::string> GCodeGenerator::generateLinearTrajectories(int count, const GenerationParams& params) const {
    std::vector<std::string> trajectories;
    trajectories.reserve(count);

    for (int i = 0; i < count; ++i) {
        LinearMove move = generateContinuousLinearMove(params);
        trajectories.push_back(formatLinearMove(move, params));
    }

    return trajectories;
}

std::vector<std::string> GCodeGenerator::generateCircularTrajectories(int count, const GenerationParams& params) const {
    std::vector<std::string> trajectories;
    trajectories.reserve(count);

    for (int i = 0; i < count; ++i) {
        CircularMove move = generateContinuousCircularMove(params);
        trajectories.push_back(formatCircularMove(move, params));
    }

    return trajectories;
}

std::vector<std::string> GCodeGenerator::generateMixedTrajectories(int count, const GenerationParams& params) const {
    std::vector<std::string> trajectories;
    trajectories.reserve(count);

    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);

    for (int i = 0; i < count; ++i) {
        if (prob_dist(rng_) < params.linear_probability) {
            LinearMove move = generateContinuousLinearMove(params);
            trajectories.push_back(formatLinearMove(move, params));
        }
        else {
            CircularMove move = generateContinuousCircularMove(params);
            trajectories.push_back(formatCircularMove(move, params));
        }
    }

    return trajectories;
}

// =============================================================================
// Continuous Motion Generation Methods
// =============================================================================
GCodeGenerator::LinearMove GCodeGenerator::generateContinuousLinearMove(const GenerationParams& params) const {
    LinearMove move;

    // Use 90% of max trajectory time for linear moves (safety buffer for accel/decel)
    double linear_time_limit = 0.9 * params.max_trajectory_time;

    // Set start position - always start from 0,0,0 for first trajectory
    if (hasLastPosition_) {
        move.start = lastPosition_;  // Continue from last position
    }
    else {
        move.start = Eigen::Vector3d(0.0, 0.0, 0.0);  // First trajectory starts at origin
        std::cout << "First trajectory starting at origin (0,0,0)\n";
    }

    // Generate end position
    move.end = generateNextPosition(move.start, params);

    // Calculate distance and feedrate
    double distance = (move.end - move.start).norm();
    move.feedrate = calculateOptimalFeedrate(distance, linear_time_limit);
    move.estimated_time = estimateTrajectoryTime(distance, move.feedrate);

    // Update position tracking
    lastPosition_ = move.end;
    hasLastPosition_ = true;

    return move;
}

GCodeGenerator::CircularMove GCodeGenerator::generateContinuousCircularMove(const GenerationParams& params) const {
    // Select plane for circular move
    std::string plane = selectRandomPlane(params);

    // Generate continuous circular move in selected plane
    return generateContinuousCircularInPlane(plane, params);
}

GCodeGenerator::CircularMove GCodeGenerator::generateContinuousCircularInPlane(
    const std::string& plane, const GenerationParams& params) const {

    const int max_arc_attempts = 5;  // Try 5 times to generate valid arc

    for (int attempt = 0; attempt < max_arc_attempts; ++attempt) {
        std::cout << "Attempting to generate circular arc (attempt " << (attempt + 1)
            << "/" << max_arc_attempts << ")" << std::endl;

        // Generate new random parameters for each attempt
        CircularMove move = generateSingleArcAttempt(plane, params);

        if (move.isValid) {
            std::cout << "Successfully generated valid circular arc" << std::endl;
            return move;
        }

        std::cout << "Arc attempt " << (attempt + 1) << " failed validation" << std::endl;
    }

    // All arc attempts failed - generate fallback linear move
    std::cout << "All arc attempts failed, generating fallback linear move" << std::endl;
    return generateFallbackLinearMove(params);
}

// =============================================================================
// NEW: Analytical Arc Generation and Validation
// =============================================================================

GCodeGenerator::CircularMove GCodeGenerator::generateSingleArcAttempt(
    const std::string& plane, const GenerationParams& params) const {

    CircularMove move;
    move.plane_code = plane;
    move.isValid = false;

    // Set start position - continue from last position or start at origin
    if (hasLastPosition_) {
        move.start = lastPosition_;
    }
    else {
        move.start = Eigen::Vector3d(0.0, 0.0, 0.0);  // Always start at origin
        std::cout << "Starting first trajectory at origin (0,0,0)" << std::endl;
    }

    // Generate fresh random parameters for this attempt
    std::uniform_real_distribution<double> radius_dist(params.min_radius, params.max_radius);
    std::uniform_real_distribution<double> angle_dist(params.min_arc_angle, params.max_arc_angle);
    std::uniform_real_distribution<double> dir_dist(0.0, 1.0);
    std::uniform_real_distribution<double> geom_dist(0.0, 1.0);
    std::uniform_real_distribution<double> start_angle_dist(0.0, 2.0 * M_PI);

    double radius = radius_dist(rng_);
    double arc_angle_deg = angle_dist(rng_);
    double arc_angle = arc_angle_deg * M_PI / 180.0;
    double start_angle = start_angle_dist(rng_);

    move.direction = (dir_dist(rng_) < 0.5) ? CircularDirection::CLOCKWISE : CircularDirection::COUNTERCLOCKWISE;
    move.geometry = (geom_dist(rng_) < 0.5) ? ArcGeometry::CONCAVE : ArcGeometry::CONVEX;

    // Calculate arc geometry
    Eigen::Vector3d center_point, end_point, extreme_point;

    if (!calculateArcGeometry(plane, move.start, radius, start_angle, arc_angle,
        move.direction, move.geometry, center_point, end_point,
        extreme_point, move.center_offset)) {
        std::cout << "Arc geometry calculation failed" << std::endl;
        return move;  // isValid = false
    }

    // Validate arc radii consistency
    if (!validateArcRadii(move.start, end_point, center_point, radius)) {
        std::cout << "Arc radius validation failed" << std::endl;
        return move;  // isValid = false
    }

    // Store arc parameters for analytical validation
    move.center = center_point;
    move.radius = radius;
    move.start_angle = start_angle;
    move.arc_angle = arc_angle;

    // NEW: Analytical validation instead of sampling
    if (!validateCompleteArcPath(center_point, radius, start_angle, arc_angle, plane)) {
        std::cout << "Arc analytical boundary validation failed" << std::endl;
        return move;  // isValid = false
    }

    // All validations passed - finalize the arc
    move.end = end_point;

    // Calculate feedrate and timing
    double arc_length = radius * std::abs(arc_angle);
    double arc_time_limit = 0.9 * params.max_trajectory_time;
    move.feedrate = calculateOptimalFeedrate(arc_length, arc_time_limit);
    move.estimated_time = estimateTrajectoryTime(arc_length, move.feedrate);
    move.arc_depth = radius * (1.0 - std::cos(std::abs(arc_angle) / 2.0));  // Correct sagitta calculation

    // Update position tracking
    lastPosition_ = move.end;
    hasLastPosition_ = true;
    move.isValid = true;

    std::cout << "Generated valid arc: radius=" << radius << "mm, angle=" << arc_angle_deg
        << "°, depth=" << move.arc_depth << "mm, plane=" << plane << std::endl;

    return move;
}

// =============================================================================
// NEW: 3D Analytical Arc Validation
// =============================================================================

bool GCodeGenerator::validateCompleteArcPath(const Eigen::Vector3d& center,
    double radius,
    double start_angle,
    double arc_angle,
    const std::string& plane) const {

    // Get 3D plane definition
    PlaneDefinition planeInfo = getPlaneDefinition(plane);

    // Get effective workspace bounds with safety margin
    Eigen::Vector3d effectiveMin = constraints_.min_position + Eigen::Vector3d(10.0, 10.0, 10.0);
    Eigen::Vector3d effectiveMax = constraints_.max_position - Eigen::Vector3d(10.0, 10.0, 10.0);

    // Check intersection with all 6 workspace boundary planes
    for (int axis = 0; axis < 3; ++axis) {
        // Check min boundary for this axis
        if (doesArcIntersect3DBoundary(center, radius, start_angle, arc_angle,
            planeInfo, axis, effectiveMin(axis), true)) {
            return false;
        }

        // Check max boundary for this axis
        if (doesArcIntersect3DBoundary(center, radius, start_angle, arc_angle,
            planeInfo, axis, effectiveMax(axis), false)) {
            return false;
        }
    }

    return true;  // No intersections - arc is valid
}

GCodeGenerator::PlaneDefinition GCodeGenerator::getPlaneDefinition(const std::string& plane) const {
    PlaneDefinition def;
    def.plane_code = plane;

    if (plane == "G17") {  // XY plane
        def.u_axis = Eigen::Vector3d(1, 0, 0);    // X axis
        def.v_axis = Eigen::Vector3d(0, 1, 0);    // Y axis  
        def.normal = Eigen::Vector3d(0, 0, 1);    // Z normal
        def.u_index = 0; def.v_index = 1;
    }
    else if (plane == "G18") {  // ZX plane
        def.u_axis = Eigen::Vector3d(0, 0, 1);    // Z axis
        def.v_axis = Eigen::Vector3d(1, 0, 0);    // X axis
        def.normal = Eigen::Vector3d(0, 1, 0);    // Y normal  
        def.u_index = 2; def.v_index = 0;
    }
    else {  // G19 - YZ plane
        def.u_axis = Eigen::Vector3d(0, 1, 0);    // Y axis
        def.v_axis = Eigen::Vector3d(0, 0, 1);    // Z axis
        def.normal = Eigen::Vector3d(1, 0, 0);    // X normal
        def.u_index = 1; def.v_index = 2;
    }

    return def;
}

bool GCodeGenerator::doesArcIntersect3DBoundary(const Eigen::Vector3d& center,
    double radius,
    double start_angle,
    double arc_angle,
    const PlaneDefinition& plane,
    int boundaryAxis,
    double boundaryValue,
    bool isMinBoundary) const {

    // Case 1: Boundary plane is parallel to the arc plane
    if (std::abs(plane.normal(boundaryAxis)) < 1e-9) {
        // Arc is entirely in a plane parallel to this boundary
        // Check if the arc plane intersects the boundary
        double arcPlanePosition = center(boundaryAxis);

        if (isMinBoundary) {
            return arcPlanePosition < boundaryValue;  // Arc plane is below min boundary
        }
        else {
            return arcPlanePosition > boundaryValue;  // Arc plane is above max boundary
        }
    }

    // Case 2: Boundary plane intersects the arc plane (typical case)
    // Distance from arc center to the intersection line
    double distanceToLine = distanceFromCenterToBoundaryLine(center, boundaryAxis,
        boundaryValue, plane);

    // If distance > radius, no intersection possible
    if (distanceToLine >= radius) {
        return false;
    }

    // Find intersection points of circle with the boundary line
    std::vector<double> intersectionAngles = findCircleLineIntersectionAngles(
        center, radius, distanceToLine, Eigen::Vector3d::Zero(), plane, boundaryAxis, boundaryValue);

    // Check if any intersection falls within our arc sweep
    for (double intersectionAngle : intersectionAngles) {
        if (isAngleInArcSweep(intersectionAngle, start_angle, arc_angle)) {
            return true;  // Found intersection within sweep
        }
    }

    return false;  // No intersections within sweep
}

double GCodeGenerator::distanceFromCenterToBoundaryLine(const Eigen::Vector3d& center,
    int boundaryAxis,
    double boundaryValue,
    const PlaneDefinition& plane) const {

    // For a plane defined by coordinate axis = constant,
    // the distance from point to plane is simply |point[axis] - constant|
    return std::abs(center(boundaryAxis) - boundaryValue);
}

std::vector<double> GCodeGenerator::findCircleLineIntersectionAngles(
    const Eigen::Vector3d& center,
    double radius,
    double distanceToLine,
    const Eigen::Vector3d& lineDirection,
    const PlaneDefinition& plane,
    int boundaryAxis,
    double boundaryValue) const {

    std::vector<double> angles;

    // Calculate the angle offset from center to intersection points
    double deltaAngle = std::acos(distanceToLine / radius);

    // Find the angle where center "looks" perpendicular to the boundary line
    double perpendicularAngle = calculatePerpendicularAngleToBoundary(
        center, boundaryAxis, boundaryValue, plane);

    // Two intersection points (circle crosses line at two points)
    angles.push_back(normalizeAngle(perpendicularAngle - deltaAngle));
    angles.push_back(normalizeAngle(perpendicularAngle + deltaAngle));

    return angles;
}

double GCodeGenerator::calculatePerpendicularAngleToBoundary(const Eigen::Vector3d& center,
    int boundaryAxis,
    double boundaryValue,
    const PlaneDefinition& plane) const {

    // Create a vector from center toward the boundary
    Eigen::Vector3d toBoundary = Eigen::Vector3d::Zero();
    toBoundary(boundaryAxis) = (boundaryValue > center(boundaryAxis)) ? 1.0 : -1.0;

    // Project this vector onto the arc plane to get 2D coordinates
    double u_component = toBoundary.dot(plane.u_axis);
    double v_component = toBoundary.dot(plane.v_axis);

    // Calculate angle in the 2D plane coordinate system
    return std::atan2(v_component, u_component);
}

bool GCodeGenerator::isAngleInArcSweep(double testAngle, double startAngle, double sweepAngle) const {
    // Normalize all angles to [0, 2π]
    testAngle = normalizeAngle(testAngle);
    startAngle = normalizeAngle(startAngle);

    // Calculate end angle
    double endAngle = normalizeAngle(startAngle + sweepAngle);

    // Check if test angle is within the sweep
    if (sweepAngle >= 0) {  // Counterclockwise sweep
        if (startAngle <= endAngle) {
            // Normal case: no wraparound
            return (testAngle >= startAngle && testAngle <= endAngle);
        }
        else {
            // Wraparound case: sweep crosses 0/2π boundary
            return (testAngle >= startAngle || testAngle <= endAngle);
        }
    }
    else {  // Clockwise sweep
        sweepAngle = -sweepAngle;  // Make positive for calculation
        endAngle = normalizeAngle(startAngle - sweepAngle);

        if (startAngle >= endAngle) {
            // Normal case: no wraparound
            return (testAngle <= startAngle && testAngle >= endAngle);
        }
        else {
            // Wraparound case: sweep crosses 0/2π boundary  
            return (testAngle <= startAngle || testAngle >= endAngle);
        }
    }
}

double GCodeGenerator::normalizeAngle(double angle) const {
    while (angle < 0) angle += 2.0 * M_PI;
    while (angle >= 2.0 * M_PI) angle -= 2.0 * M_PI;
    return angle;
}

// =============================================================================
// Position Generation Helper
// =============================================================================
Eigen::Vector3d GCodeGenerator::generateNextPosition(const Eigen::Vector3d& currentPos, const GenerationParams& params) const {
    const int max_attempts = 50;

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        // Generate random direction with bias toward workspace center if near edges
        std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);
        std::uniform_real_distribution<double> elevation_dist(-M_PI / 6, M_PI / 6);  // ±30 degrees
        std::uniform_real_distribution<double> distance_dist(params.min_move_distance, params.max_move_distance);

        double azimuth = angle_dist(rng_);
        double elevation = elevation_dist(rng_);
        double distance = distance_dist(rng_);

        // Calculate workspace center for bias calculation
        Eigen::Vector3d workspaceCenter = (constraints_.max_position + constraints_.min_position) / 2.0;

        // If near workspace edges, bias direction toward center
        if (!isPositionValid(currentPos + Eigen::Vector3d(50, 50, 50))) {
            Eigen::Vector3d toCenter = (workspaceCenter - currentPos).normalized();
            // Blend random direction with center direction
            Eigen::Vector3d randomDir(std::cos(elevation) * std::cos(azimuth),
                std::cos(elevation) * std::sin(azimuth),
                std::sin(elevation));
            Eigen::Vector3d direction = (randomDir + toCenter * 0.5).normalized();
            Eigen::Vector3d endPos = currentPos + direction * distance;

            if (isPositionValid(endPos)) {
                return endPos;
            }
        }
        else {
            // Normal random direction
            Eigen::Vector3d direction(std::cos(elevation) * std::cos(azimuth),
                std::cos(elevation) * std::sin(azimuth),
                std::sin(elevation));
            Eigen::Vector3d endPos = currentPos + direction * distance;

            if (isPositionValid(endPos)) {
                return endPos;
            }
        }
    }

    // Fallback - small move toward workspace center
    Eigen::Vector3d workspaceCenter = (constraints_.max_position + constraints_.min_position) / 2.0;
    Eigen::Vector3d toCenter = (workspaceCenter - currentPos).normalized();
    return currentPos + toCenter * params.min_move_distance;
}

// =============================================================================
// G-code Formatting Methods
// =============================================================================
std::string GCodeGenerator::formatLinearMove(const LinearMove& move, const GenerationParams& params) const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);

    // G4 dwell command (between trajectories)
    if (params.use_dwell_commands) {
        ss << formatDwellCommand(params.dwell_time) << std::endl;
    }

    // G1 linear move from current position to end position
    ss << "G1 X" << move.end.x() << " Y" << move.end.y() << " Z" << move.end.z()
        << " F" << static_cast<int>(move.feedrate);

    return ss.str();
}

std::string GCodeGenerator::formatCircularMove(const CircularMove& move, const GenerationParams& params) const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);

    // Check if this is actually a linear fallback
    if (move.isLinearFallback) {
        std::cout << "Formatting fallback linear move as G1" << std::endl;

        // G4 dwell command (between trajectories)
        if (params.use_dwell_commands) {
            ss << formatDwellCommand(params.dwell_time) << std::endl;
        }

        // Format as G1 linear move
        ss << "G1 X" << move.end.x() << " Y" << move.end.y() << " Z" << move.end.z()
            << " F" << static_cast<int>(move.feedrate);

        return ss.str();
    }

    // Set plane if not XY
    if (move.plane_code != current_plane_) {
        ss << move.plane_code << std::endl;
        setCurrentPlane(move.plane_code);  // Update current plane
    }

    // G4 dwell command (between trajectories)
    if (params.use_dwell_commands) {
        ss << formatDwellCommand(params.dwell_time) << std::endl;
    }

    // Add circular movement command
    std::string move_code = (move.direction == CircularDirection::CLOCKWISE) ? "G2" : "G3";
    ss << move_code << " X" << move.end.x() << " Y" << move.end.y() << " Z" << move.end.z();

    // Add center offsets based on plane
    if (move.plane_code == "G17") {  // XY plane
        ss << " I" << move.center_offset.x() << " J" << move.center_offset.y();
    }
    else if (move.plane_code == "G18") {  // ZX plane
        ss << " I" << move.center_offset.x() << " K" << move.center_offset.z();
    }
    else {  // G19 - YZ plane
        ss << " J" << move.center_offset.y() << " K" << move.center_offset.z();
    }

    ss << " F" << static_cast<int>(move.feedrate);
    return ss.str();
}

std::string GCodeGenerator::formatDwellCommand(double dwell_time) const {
    std::ostringstream ss;
    ss << "G4 P" << static_cast<int>(dwell_time * 1000);  // Convert to milliseconds
    return ss.str();
}

std::string GCodeGenerator::formatHeader() const {
    return "G90 G21 G17 G54";  // Absolute positioning, metric units, XY plane
}

// =============================================================================
// Fallback Linear Move Generation
// =============================================================================
GCodeGenerator::CircularMove GCodeGenerator::generateFallbackLinearMove(const GenerationParams& params) const {

    std::cout << "Generating fallback linear move disguised as arc structure" << std::endl;

    // Generate a linear move but package it in CircularMove structure for consistency
    LinearMove linear = generateContinuousLinearMove(params);

    CircularMove fallback;
    fallback.start = linear.start;
    fallback.end = linear.end;
    fallback.feedrate = linear.feedrate;
    fallback.estimated_time = linear.estimated_time;
    fallback.direction = CircularDirection::CLOCKWISE;  // Dummy values
    fallback.geometry = ArcGeometry::CONVEX;
    fallback.plane_code = "G17";
    fallback.center_offset = Eigen::Vector3d::Zero();  // Will be formatted as linear move
    fallback.arc_depth = 0.0;  // Linear move has no arc depth
    fallback.isValid = true;
    fallback.isLinearFallback = true;  // Flag to indicate this should be formatted as G1

    // Update position tracking
    lastPosition_ = fallback.end;
    hasLastPosition_ = true;

    return fallback;
}

// =============================================================================
// Utility and Helper Methods
// =============================================================================
Eigen::Vector3d GCodeGenerator::generateRandomPosition() const {
    std::uniform_real_distribution<double> x_dist(constraints_.min_position.x() + 10,
        constraints_.max_position.x() - 10);
    std::uniform_real_distribution<double> y_dist(constraints_.min_position.y() + 10,
        constraints_.max_position.y() - 10);
    std::uniform_real_distribution<double> z_dist(constraints_.min_position.z() + 10,
        constraints_.max_position.z() - 10);

    return Eigen::Vector3d(x_dist(rng_), y_dist(rng_), z_dist(rng_));
}

double GCodeGenerator::calculateOptimalFeedrate(double distance, double max_time) const {
    // Calculate feedrate in mm/min to complete trajectory within max_time
    double feedrate_mm_per_sec = distance / max_time;
    double feedrate_mm_per_min = feedrate_mm_per_sec * 60.0;

    // Apply some randomness (±20%) while staying within time constraint
    std::uniform_real_distribution<double> variance_dist(0.8, 1.2);
    feedrate_mm_per_min *= variance_dist(rng_);

    // Ensure we don't exceed machine velocity limits (convert to mm/min)
    double max_machine_feedrate = constraints_.max_velocity.maxCoeff() * 60.0;
    feedrate_mm_per_min = std::min(feedrate_mm_per_min, max_machine_feedrate);

    // Ensure minimum practical feedrate
    feedrate_mm_per_min = std::max(feedrate_mm_per_min, 100.0);

    return feedrate_mm_per_min;
}

std::string GCodeGenerator::selectRandomPlane(const GenerationParams& params) const {
    std::uniform_real_distribution<double> plane_dist(0.0, 1.0);
    double rand_val = plane_dist(rng_);

    if (rand_val < params.xy_plane_probability) {
        return "G17";  // XY plane
    }
    else if (rand_val < params.xy_plane_probability + params.xz_plane_probability) {
        return "G18";  // ZX plane
    }
    else {
        return "G19";  // YZ plane
    }
}

bool GCodeGenerator::isPositionValid(const Eigen::Vector3d& pos) const {
    Eigen::Vector3d margin = Eigen::Vector3d(10.0, 10.0, 10.0);
    return (pos.array() >= constraints_.min_position.array() + margin.array()).all() &&
        (pos.array() <= constraints_.max_position.array() - margin.array()).all();
}

bool GCodeGenerator::validateArcRadii(const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector3d& center,
    double expected_radius) const {

    double start_radius = (start - center).norm();
    double end_radius = (end - center).norm();

    const double tolerance = 1e-3;  // 1 micron tolerance

    if (std::abs(start_radius - expected_radius) > tolerance) {
        std::cerr << "Warning: Start radius mismatch: expected " << expected_radius
            << ", got " << start_radius << std::endl;
        return false;
    }

    if (std::abs(end_radius - expected_radius) > tolerance) {
        std::cerr << "Warning: End radius mismatch: expected " << expected_radius
            << ", got " << end_radius << std::endl;
        return false;
    }

    if (std::abs(start_radius - end_radius) > tolerance) {
        std::cerr << "Warning: Start and end radii differ: start=" << start_radius
            << ", end=" << end_radius << std::endl;
        return false;
    }

    return true;
}

// =============================================================================
// Configuration and State Management
// =============================================================================
void GCodeGenerator::setConstraints(const MachineConstraints& constraints) {
    constraints_ = constraints;
}

void GCodeGenerator::setSeed(unsigned int seed) {
    current_seed_ = seed;
    rng_.seed(current_seed_);
}

// =============================================================================
// Summary and Documentation
// =============================================================================
std::string GCodeGenerator::getGenerationSummary(const GenerationParams& params) const {
    std::ostringstream summary;

    summary << "=== G-Code Generation Summary ===" << std::endl;
    summary << "Number of trajectories: " << params.num_trajectories << std::endl;
    summary << "Max trajectory time: " << params.max_trajectory_time << " seconds" << std::endl;
    summary << "Initial dwell time: 10 seconds (automatic)" << std::endl;
    summary << "Inter-trajectory dwell time: " << params.dwell_time << " seconds" << std::endl;

    // Trajectory type description
    switch (params.trajectory_type) {
    case TrajectoryType::LINEAR_ONLY:
        summary << "Trajectory type: Linear only" << std::endl;
        break;
    case TrajectoryType::CIRCULAR_ONLY:
        summary << "Trajectory type: Circular only" << std::endl;
        break;
    case TrajectoryType::MIXED:
        summary << "Trajectory type: Mixed (" << static_cast<int>(params.linear_probability * 100) << "% linear)" << std::endl;
        break;
    }

    summary << "Arc geometry: Mixed (concave/convex)" << std::endl;

    // Workspace info
    summary << "Workspace: X[" << constraints_.min_position.x() << ", " << constraints_.max_position.x()
        << "] Y[" << constraints_.min_position.y() << ", " << constraints_.max_position.y()
        << "] Z[" << constraints_.min_position.z() << ", " << constraints_.max_position.z() << "]" << std::endl;

    summary << "Max velocity: " << constraints_.max_velocity.x() << " "
        << constraints_.max_velocity.y() << " " << constraints_.max_velocity.z() << " mm/s" << std::endl;

    summary << "Arc radius range: " << params.min_radius << " - " << params.max_radius << " mm" << std::endl;
    summary << "Min arc depth: " << params.min_arc_depth << " mm" << std::endl;
    summary << "Rapid positioning: NO (continuous motion only)" << std::endl;
    summary << "Dwell commands: " << (params.use_dwell_commands ? "Yes" : "No") << " (between trajectories)" << std::endl;
    summary << "Initial dwell: YES (10 seconds automatic)" << std::endl;

    // Plane distribution
    summary << "Plane distribution: XY(" << static_cast<int>(params.xy_plane_probability * 100)
        << "%) XZ(" << static_cast<int>(params.xz_plane_probability * 100)
        << "%) YZ(" << static_cast<int>(params.yz_plane_probability * 100) << "%)" << std::endl;

    summary << "=================================" << std::endl;

    return summary.str();
}

std::string GCodeGenerator::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &time_t);
#else
    localtime_r(&time_t, &tm_buf);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S_")
        << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

std::string GCodeGenerator::generateSummaryFilename(const std::string& directory) const {
    return directory + "/summary_" + getCurrentTimestamp() + ".txt";
}

void GCodeGenerator::writeSummaryToFile(const std::string& summary_content, const std::string& filepath) const {
    try {
        std::filesystem::create_directories(std::filesystem::path(filepath).parent_path());
        std::ofstream file(filepath);
        if (file.is_open()) {
            file << summary_content;
            file.close();
        }
        else {
            std::cerr << "Warning: Could not write summary file: " << filepath << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Warning: Exception writing summary file: " << e.what() << std::endl;
    }
}

// =============================================================================
// Static Utility Methods
// =============================================================================
bool GCodeGenerator::validateGCodeSyntax(const std::string& gcode_line) {
    if (gcode_line.empty()) return false;

    // Should start with G, M, or be a comment
    char first_char = std::toupper(gcode_line[0]);
    return (first_char == 'G' || first_char == 'M' || first_char == ';' || first_char == '(');
}

double GCodeGenerator::estimateTrajectoryTime(double distance, double feedrate) {
    if (feedrate <= 0) return 0.0;

    // Convert feedrate from mm/min to mm/s
    double feedrate_mm_per_sec = feedrate / 60.0;

    // Simple time calculation (ignoring acceleration/deceleration)
    return distance / feedrate_mm_per_sec;
}

double GCodeGenerator::estimateMinFeedrate(double distance, double time) {
    if (time <= 0) return 1000.0;  // Default feedrate

    // Calculate required feedrate in mm/min
    double required_feedrate_mm_per_sec = distance / time;
    return required_feedrate_mm_per_sec * 60.0;  // Convert to mm/min
}

// =============================================================================
// Arc Geometry Calculation Methods (Existing Implementation)
// =============================================================================

bool GCodeGenerator::calculateArcGeometry(const std::string& plane,
    const Eigen::Vector3d& start_point,
    double radius,
    double start_angle,
    double arc_angle,
    CircularDirection direction,
    ArcGeometry geometry,
    Eigen::Vector3d& center_point,
    Eigen::Vector3d& end_point,
    Eigen::Vector3d& extreme_point,
    Eigen::Vector3d& center_offset) const {

    // Get plane indices
    int idx1, idx2, static_idx;
    if (plane == "G17") {      // XY plane
        idx1 = 0; idx2 = 1; static_idx = 2;
    }
    else if (plane == "G18") { // ZX plane  
        idx1 = 0; idx2 = 2; static_idx = 1;
    }
    else {                   // G19 - YZ plane
        idx1 = 1; idx2 = 2; static_idx = 0;
    }

    // Calculate center position using consistent geometry
    Eigen::Vector3d center_2d = calculateCenterPosition2D(start_angle, radius, direction, geometry);

    // Create 3D center point
    center_point = start_point;
    center_point[idx1] += center_2d.x();
    center_point[idx2] += center_2d.y();

    // Calculate end point using EXACT radius constraint
    double end_angle = calculateEndAngle(start_angle, arc_angle, direction, geometry);

    end_point = center_point;
    end_point[idx1] += radius * std::cos(end_angle);
    end_point[idx2] += radius * std::sin(end_angle);
    end_point[static_idx] = start_point[static_idx]; // Keep static axis unchanged

    // Calculate extreme point (midpoint of arc)
    double mid_angle = calculateMidAngle(start_angle, end_angle, direction);

    extreme_point = center_point;
    extreme_point[idx1] += radius * std::cos(mid_angle);
    extreme_point[idx2] += radius * std::sin(mid_angle);
    extreme_point[static_idx] = start_point[static_idx];

    // Calculate center offset (I, J, K values)
    center_offset = Eigen::Vector3d::Zero();
    center_offset[idx1] = center_point[idx1] - start_point[idx1];
    center_offset[idx2] = center_point[idx2] - start_point[idx2];
    // Static axis offset is always 0

    return true;
}

Eigen::Vector3d GCodeGenerator::calculateCenterPosition2D(double start_angle,
    double radius,
    CircularDirection direction,
    ArcGeometry geometry) const {
    // Calculate perpendicular direction based on geometry and direction
    double perpendicular_angle;

    if ((direction == CircularDirection::COUNTERCLOCKWISE && geometry == ArcGeometry::CONVEX) ||
        (direction == CircularDirection::CLOCKWISE && geometry == ArcGeometry::CONCAVE)) {
        // Center is 90 degrees counterclockwise from start direction
        perpendicular_angle = start_angle + M_PI / 2.0;
    }
    else {
        // Center is 90 degrees clockwise from start direction  
        perpendicular_angle = start_angle - M_PI / 2.0;
    }

    // Calculate center offset from start point
    Eigen::Vector3d center_offset_2d;
    center_offset_2d.x() = radius * std::cos(perpendicular_angle);
    center_offset_2d.y() = radius * std::sin(perpendicular_angle);
    center_offset_2d.z() = 0.0;

    return center_offset_2d;
}

double GCodeGenerator::calculateEndAngle(double start_angle,
    double arc_angle,
    CircularDirection direction,
    ArcGeometry geometry) const {
    double end_angle;

    if ((direction == CircularDirection::COUNTERCLOCKWISE && geometry == ArcGeometry::CONVEX) ||
        (direction == CircularDirection::CLOCKWISE && geometry == ArcGeometry::CONCAVE)) {
        // Angle increases (counterclockwise motion)
        end_angle = start_angle + arc_angle;
    }
    else {
        // Angle decreases (clockwise motion)
        end_angle = start_angle - arc_angle;
    }

    // Normalize angle to [0, 2π]
    while (end_angle < 0) end_angle += 2.0 * M_PI;
    while (end_angle >= 2.0 * M_PI) end_angle -= 2.0 * M_PI;

    return end_angle;
}

double GCodeGenerator::calculateMidAngle(double start_angle,
    double end_angle,
    CircularDirection direction) const {
    double mid_angle;

    if (direction == CircularDirection::COUNTERCLOCKWISE) {
        if (end_angle > start_angle) {
            mid_angle = (start_angle + end_angle) / 2.0;
        }
        else {
            // Crosses 0/2π boundary
            mid_angle = (start_angle + end_angle + 2.0 * M_PI) / 2.0;
            if (mid_angle >= 2.0 * M_PI) mid_angle -= 2.0 * M_PI;
        }
    }
    else { // CLOCKWISE
        if (start_angle > end_angle) {
            mid_angle = (start_angle + end_angle) / 2.0;
        }
        else {
            // Crosses 0/2π boundary
            mid_angle = (start_angle + end_angle + 2.0 * M_PI) / 2.0;
            if (mid_angle >= 2.0 * M_PI) mid_angle -= 2.0 * M_PI;
        }
    }
    return mid_angle;
}

// Legacy methods for compatibility
GCodeGenerator::LinearMove GCodeGenerator::generateLinearMove(const GenerationParams& params) const {
    return generateContinuousLinearMove(params);
}

GCodeGenerator::CircularMove GCodeGenerator::generateCircularMove(const GenerationParams& params) const {
    return generateContinuousCircularMove(params);
}

GCodeGenerator::CircularMove GCodeGenerator::generateCircularInPlane(const std::string& plane, const GenerationParams& params) const {
    return generateContinuousCircularInPlane(plane, params);
}

GCodeGenerator::CircularMove GCodeGenerator::createFallbackArc(const GenerationParams& params) const {
    CircularMove move;

    // Create a simple, guaranteed-valid arc
    move.start = Eigen::Vector3d(0.0, 0.0, 0.0);
    move.end = Eigen::Vector3d(2.0 * params.min_radius, 0.0, 0.0);
    move.center_offset = Eigen::Vector3d(params.min_radius, 0.0, 0.0);
    move.feedrate = 1000.0;
    move.arc_depth = params.min_arc_depth;
    move.direction = CircularDirection::CLOCKWISE;
    move.geometry = ArcGeometry::CONCAVE;
    move.estimated_time = 0.1;
    move.plane_code = "G17";

    return move;
}