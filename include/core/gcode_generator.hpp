#pragma once
#include <string>
#include <vector>
#include <random>
#include <fstream>
#include <Eigen/Dense>
#include "core/trajectory_types.hpp"  // Use existing MachineConstraints

enum class TrajectoryType {
    LINEAR_ONLY,
    CIRCULAR_ONLY,
    MIXED
};

enum class CircularDirection {
    CLOCKWISE,         // G2
    COUNTERCLOCKWISE,  // G3
    RANDOM
};

enum class ArcGeometry {
    CONCAVE,    // Arc curves inward
    CONVEX,     // Arc curves outward
    RANDOM
};

struct GenerationParams {
    int num_trajectories = 10;
    TrajectoryType trajectory_type = TrajectoryType::MIXED;
    CircularDirection circular_direction = CircularDirection::RANDOM;
    ArcGeometry arc_geometry = ArcGeometry::RANDOM;

    // Trajectory timing constraints
    double max_trajectory_time = 2.0;  // seconds
    double dwell_time = 0.5;          // seconds (G4 P500 = 500ms) - between trajectories

    // Probability distribution for mixed mode (0.0 to 1.0)
    double linear_probability = 0.5;  // 60% linear, 40% circular

    // Plane distribution for circular moves
    double xy_plane_probability = 0.34;  // 50% XY plane
    double xz_plane_probability = 0.33;  // 30% XZ plane
    double yz_plane_probability = 0.33;  // 20% YZ plane

    // Arc constraints
    Eigen::Vector3d margin = Eigen::Vector3d(10.0, 10.0, 10.0);
    double min_radius = 5.0;           // mm
    double max_radius = 50.0;          // mm
    double min_arc_depth = 0.005;      // mm (minimum depth for arcs)
    double min_arc_angle = 30.0;       // degrees
    double max_arc_angle = 180.0;      // degrees (hard limit: no arc > 180°)

    // Move distance constraints for linear moves
    double min_move_distance = 10.0;   // mm
    double max_move_distance = 200.0;  // mm

    // Dwell commands between trajectories
    bool use_dwell_commands = true;

    // Random seed (0 for random seed)
    unsigned int seed = 0;

    // Summary output options
    std::string summary_output_directory = "./";  // Directory for summary files
    bool write_summary_to_file = true;            // Whether to write summary to file

    bool isValid() const {
        return num_trajectories > 0 &&
            max_trajectory_time > 0.0 &&
            dwell_time >= 0.0 &&
            linear_probability >= 0.0 && linear_probability <= 1.0 &&
            std::abs(xy_plane_probability + xz_plane_probability + yz_plane_probability - 1.0) < 1e-6 &&
            min_radius < max_radius &&
            min_arc_depth > 0.0 &&
            min_arc_angle < max_arc_angle;
    }
};

class GCodeGenerator {
public:
    // Helper structures for generation
    struct LinearMove {
        Eigen::Vector3d start;
        Eigen::Vector3d end;
        double feedrate;
        double estimated_time;
    };

    struct CircularMove {
        Eigen::Vector3d start;
        Eigen::Vector3d end;
        Eigen::Vector3d center_offset;  // I, J, K values
        double feedrate;
        double estimated_time;
        CircularDirection direction;
        ArcGeometry geometry;
        std::string plane_code;  // G17, G18, or G19
        double arc_depth;       // Depth of the arc

        // Enhanced validation and fallback fields
        bool isValid = false;           // Indicates if arc is valid
        bool isLinearFallback = false;  // Indicates if this is a linear fallback

        // Additional fields for analytical validation
        Eigen::Vector3d center;         // Actual 3D center position
        double radius = 0.0;            // Arc radius
        double start_angle = 0.0;       // Start angle in 2D plane
        double arc_angle = 0.0;         // Arc sweep angle
    };

private:
    MachineConstraints constraints_;  // Use existing constraints struct
    mutable std::mt19937 rng_;
    unsigned int current_seed_;
    mutable std::string current_plane_ = "G17";  // Current plane for circular moves
    mutable Eigen::Vector3d lastPosition_;       // Track last position for continuous motion
    mutable bool hasLastPosition_ = false;       // Track if we have a previous position

    // 3D Plane Definition for analytical validation
    struct PlaneDefinition {
        Eigen::Vector3d u_axis;     // Primary axis of the plane (unit vector)
        Eigen::Vector3d v_axis;     // Secondary axis of the plane (unit vector)  
        Eigen::Vector3d normal;     // Normal to the plane (unit vector)
        int u_index, v_index;       // Which coordinate axes (0=X, 1=Y, 2=Z)
        std::string plane_code;
    };

    // Random generation methods
    Eigen::Vector3d generateRandomPosition() const;
    Eigen::Vector3d generateNextPosition(const Eigen::Vector3d& currentPos, const GenerationParams& params) const;
    double calculateOptimalFeedrate(double distance, double max_time) const;
    LinearMove generateLinearMove(const GenerationParams& params) const;
    LinearMove generateContinuousLinearMove(const GenerationParams& params) const;
    CircularMove generateCircularMove(const GenerationParams& params) const;
    CircularMove generateContinuousCircularMove(const GenerationParams& params) const;
    std::string selectRandomPlane(const GenerationParams& params) const;
    CircularMove generateCircularInPlane(const std::string& plane, const GenerationParams& params) const;
    CircularMove generateContinuousCircularInPlane(const std::string& plane, const GenerationParams& params) const;

    // Enhanced arc generation with analytical validation
    CircularMove generateSingleArcAttempt(const std::string& plane, const GenerationParams& params) const;
    CircularMove generateFallbackLinearMove(const GenerationParams& params) const;

    // NEW: Analytical validation methods
    bool validateCompleteArcPath(const Eigen::Vector3d& center,
        double radius,
        double start_angle,
        double arc_angle,
        const std::string& plane) const;

    PlaneDefinition getPlaneDefinition(const std::string& plane) const;

    bool doesArcIntersect3DBoundary(const Eigen::Vector3d& center,
        double radius,
        double start_angle,
        double arc_angle,
        const PlaneDefinition& plane,
        int boundaryAxis,
        double boundaryValue,
        bool isMinBoundary) const;

    Eigen::Vector3d findBoundaryLineInArcPlane(const PlaneDefinition& plane,
        int boundaryAxis) const;

    double distanceFromCenterToBoundaryLine(const Eigen::Vector3d& center,
        int boundaryAxis,
        double boundaryValue,
        const PlaneDefinition& plane) const;

    std::vector<double> findCircleLineIntersectionAngles(
        const Eigen::Vector3d& center,
        double radius,
        double distanceToLine,
        const Eigen::Vector3d& lineDirection,
        const PlaneDefinition& plane,
        int boundaryAxis,
        double boundaryValue) const;

    double calculatePerpendicularAngleToBoundary(const Eigen::Vector3d& center,
        int boundaryAxis,
        double boundaryValue,
        const PlaneDefinition& plane) const;

    bool isAngleInArcSweep(double testAngle, double startAngle, double sweepAngle) const;
    double normalizeAngle(double angle) const;

    // Validation methods
    bool isPositionValid(const Eigen::Vector3d& pos) const;
    bool validateArcRadii(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
        const Eigen::Vector3d& center, double expected_radius) const;

    // G-code formatting methods
    std::string formatLinearMove(const LinearMove& move, const GenerationParams& params) const;
    std::string formatCircularMove(const CircularMove& move, const GenerationParams& params) const;
    std::string formatDwellCommand(double dwell_time) const;
    std::string formatHeader() const;

    // Summary and utility methods
    std::string getCurrentTimestamp() const;
    std::string generateSummaryFilename(const std::string& directory) const;
    void writeSummaryToFile(const std::string& summary_content, const std::string& filepath) const;

    // Arc geometry calculation methods
    bool calculateArcGeometry(const std::string& plane,
        const Eigen::Vector3d& start_point,
        double radius,
        double start_angle,
        double arc_angle,
        CircularDirection direction,
        ArcGeometry geometry,
        Eigen::Vector3d& center_point,
        Eigen::Vector3d& end_point,
        Eigen::Vector3d& extreme_point,
        Eigen::Vector3d& center_offset) const;

    Eigen::Vector3d calculateCenterPosition2D(double start_angle,
        double radius,
        CircularDirection direction,
        ArcGeometry geometry) const;

    double calculateEndAngle(double start_angle,
        double arc_angle,
        CircularDirection direction,
        ArcGeometry geometry) const;

    double calculateMidAngle(double start_angle,
        double end_angle,
        CircularDirection direction) const;

    CircularMove createFallbackArc(const GenerationParams& params) const;

public:
    explicit GCodeGenerator(const MachineConstraints& constraints = MachineConstraints{},
        unsigned int seed = 0);

    // Main generation function
    bool generateGCodeFile(const std::string& filename,
        const GenerationParams& params) const;

    // Individual trajectory generators
    std::vector<std::string> generateLinearTrajectories(int count, const GenerationParams& params) const;
    std::vector<std::string> generateCircularTrajectories(int count, const GenerationParams& params) const;
    std::vector<std::string> generateMixedTrajectories(int count, const GenerationParams& params) const;

    // Utility functions
    void setConstraints(const MachineConstraints& constraints);
    const MachineConstraints& getConstraints() const { return constraints_; }
    void setSeed(unsigned int seed);
    void setCurrentPlane(const std::string& plane_code) const { current_plane_ = plane_code; }
    unsigned int getCurrentSeed() const { return current_seed_; }

    // Position tracking for continuous motion
    void resetPositionTracking() const { hasLastPosition_ = false; }
    Eigen::Vector3d getLastPosition() const { return lastPosition_; }
    bool hasLastPosition() const { return hasLastPosition_; }

    // Validation and analysis
    static bool validateGCodeSyntax(const std::string& gcode_line);
    std::string getGenerationSummary(const GenerationParams& params) const;
    static double estimateTrajectoryTime(double distance, double feedrate);
    static double estimateMinFeedrate(double distance, double time);
};