#pragma once
#include <vector>


struct SCurveProfile {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    std::vector<double> jerks;
};

SCurveProfile generateSCurveProfile(
    double L,
    double J_max,
    double A_max,
    double V_max,
    double dt
);
