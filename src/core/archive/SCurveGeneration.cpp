#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "core/SCurveGeneration.hpp"

//// Generate jerk-limited S-curve profile for 1D position from 0 to L
//SCurveProfile generateSCurveProfile(
//    double L,       // total distance to move
//    double J_max,   // max jerk (units/s^3)
//    double A_max,   // max acceleration (units/s^2)
//    double V_max,   // max velocity (units/s)
//    double dt       // time step (s)
//) {
//    SCurveProfile profile;
//
//    // 1) Compute time intervals for the phases assuming full profile
//
//    double Tj = A_max / J_max;      // time to ramp acceleration (jerk phases)
//    double Ta = Tj + (V_max / A_max) - Tj;  // total accel time excluding jerk ramps simplified
//    double Td = Ta;                 // deceleration time equal accel time (symmetry)
//
//    // Distance covered during accel + decel phases
//    double Sa = A_max * (Ta - Tj) * (Ta - Tj) / 2.0 + A_max * Tj * (Ta - Tj) + J_max * Tj * Tj * Tj / 6.0;
//    double Sd = Sa;
//
//    // Distance remaining for constant velocity cruise
//    double Sc = L - (Sa + Sd);
//
//    // If Sc < 0, we cannot reach full velocity, so recalc times for triangular or trapezoidal profile
//    if (Sc < 0) {
//        // Solve for max acceleration and max velocity that fits length L
//        // Using simplified triangular profile:
//
//        // Compute reduced max acceleration that fits within length
//        A_max = std::cbrt(L * J_max * J_max / 2.0);
//        if (A_max > V_max) A_max = V_max;
//
//        Tj = A_max / J_max;
//        Ta = 2 * Tj; // accel time for triangular profile (jerk up then jerk down)
//
//        Td = Ta;
//        Sc = 0;
//        V_max = A_max * Tj; // max velocity reached (triangular)
//
//        std::cout << "Short move, using triangular profile. Adjusted A_max: " << A_max << ", V_max: " << V_max << "\n";
//    }
//
//    double Tc = (Sc > 0) ? (Sc / V_max) : 0.0;
//
//    // Total profile time
//    double T = Ta + Tc + Td;
//
//    // Number of samples
//    int N = static_cast<int>(std::ceil(T / dt));
//    profile.positions.resize(N);
//    profile.velocities.resize(N);
//    profile.accelerations.resize(N);
//
//    for (int i = 0; i < N; ++i) {
//        double t = i * dt;
//
//        double pos = 0.0;
//        double vel = 0.0;
//        double acc = 0.0;
//
//        if (t < Tj) {
//            // Phase 1: jerk positive
//            acc = J_max * t;
//            vel = 0.5 * J_max * t * t;
//            pos = (1.0 / 6.0) * J_max * t * t * t;
//        }
//        else if (t < Ta - Tj) {
//            // Phase 2: constant acceleration
//            double dt2 = t - Tj;
//            acc = A_max;
//            vel = A_max * dt2 + 0.5 * A_max * Tj;
//            pos = 0.5 * A_max * dt2 * dt2 + 0.5 * A_max * Tj * dt2 + (1.0 / 6.0) * J_max * Tj * Tj * Tj;
//        }
//        else if (t < Ta) {
//            // Phase 3: jerk negative
//            double dt3 = t - (Ta - Tj);
//            acc = A_max - J_max * dt3;
//            vel = A_max * (Ta - Tj) + 0.5 * A_max * dt3 - 0.5 * J_max * dt3 * dt3;
//            pos = L * (Sa / L) + (vel * dt); // approximation, could be refined
//        }
//        else if (t < Ta + Tc) {
//            // Phase 4: constant velocity
//            double dt4 = t - Ta;
//            acc = 0.0;
//            vel = V_max;
//            pos = (Sa)+V_max * dt4;
//        }
//        else if (t < Ta + Tc + Tj) {
//            // Phase 5: jerk negative (decel jerk down)
//            double dt5 = t - (Ta + Tc);
//            acc = -J_max * dt5;
//            vel = V_max - 0.5 * J_max * dt5 * dt5;
//            pos = (Sa + Sc) + V_max * dt5 - (1.0 / 6.0) * J_max * dt5 * dt5 * dt5;
//        }
//        else if (t < T - Tj) {
//            // Phase 6: constant negative acceleration
//            double dt6 = t - (Ta + Tc + Tj);
//            acc = -A_max;
//            vel = V_max - A_max * dt6 - 0.5 * J_max * Tj * Tj;
//            pos = (Sa + Sc) + V_max * (dt6 + Tj) - 0.5 * A_max * dt6 * dt6 - 0.5 * J_max * Tj * Tj * dt6;
//        }
//        else if (t <= T) {
//            // Phase 7: jerk positive (acc ramps to 0)
//            double dt7 = t - (T - Tj);
//            acc = -A_max + J_max * dt7;
//            vel = (J_max * Tj * Tj * Tj) / 6.0 + vel + acc * dt7;
//            pos = L - (1.0 / 6.0) * J_max * (T - t) * (T - t) * (T - t);
//        }
//        else {
//            // After profile ends, hold position
//            pos = L;
//            vel = 0.0;
//            acc = 0.0;
//        }
//
//        // Clamp pos to [0, L]
//        pos = std::min(std::max(pos, 0.0), L);
//
//        profile.positions[i] = pos;
//        profile.velocities[i] = vel;
//        profile.accelerations[i] = acc;
//    }
//
//    return profile;
//}


SCurveProfile generateSCurveProfile(
    double L,       // total distance to move
    double J_max,   // max jerk
    double A_max,   // max acceleration
    double V_max,   // max velocity
    double dt       // time step
) {
    SCurveProfile profile;

    // Initial durations
    double Tj = A_max / J_max;
    double Ta = Tj + (V_max / A_max);  // placeholder
    double Td = Ta;
    double Tv = 0;

    // Distance covered during acceleration and deceleration
    double Sa = A_max * Tj * Tj + A_max * (V_max / A_max - Tj) * Tj;
    double Sd = Sa;
    double Sc = L - (Sa + Sd);

    bool isTriangular = false;

    // Check if we can cruise at V_max, else reduce to triangular profile
    if (Sc < 0) {
        isTriangular = true;
        A_max = std::cbrt(L * J_max * J_max / 2.0);
        if (A_max > V_max) A_max = V_max;

        Tj = A_max / J_max;
        Ta = Td = 2 * Tj;
        Tv = 0;
        V_max = A_max * Tj;

        std::cout << "Using triangular profile. Adjusted A_max: " << A_max << ", V_max: " << V_max << std::endl;
    }
    else {
        Tv = Sc / V_max;
        Ta = Td = (V_max / A_max) + Tj;
    }

    double T = Ta + Tv + Td;
    int N = static_cast<int>(std::ceil(T / dt));

    profile.positions.resize(N);
    profile.velocities.resize(N);
    profile.accelerations.resize(N);
    profile.jerks.resize(N);

    // Phase boundaries
    double Ta_flat = Ta - 2 * Tj;
    double Td_flat = Td - 2 * Tj;

    double t0 = 0.0;
    double t1 = t0 + Tj;
    double t2 = t1 + Ta_flat;
    double t3 = t2 + Tj;
    double t4 = t3 + Tv;
    double t5 = t4 + Tj;
    double t6 = t5 + Td_flat;
    double t7 = t6 + Tj;


    // Integration variables
    double x = 0.0, v = 0.0, a = 0.0;

    for (int i = 0; i < N; ++i) {
        double t = i * dt;
        double j = 0.0;

        // Determine jerk phase
        if (t < t1) { j = +J_max; }                       // Phase 1: Accel ramp up
        else if (t < t2) { j = 0.0; }                     // Phase 2: Const accel
        else if (t < t3) { j = -J_max; }                  // Phase 3: Accel ramp down
        else if (t < t4) { j = 0.0; a = 0.0; }            // Phase 4: Const velocity
        else if (t < t5) { j = -J_max; }                  // Phase 5: Decel ramp down
        else if (t < t6) { j = 0.0; }                     // Phase 6: Const decel
        else if (t < t7) { j = +J_max; }                  // Phase 7: Decel ramp up
        else { j = 0.0; }                                 // After profile

        // Integrate
        a += j * dt;
        v += a * dt;
        x += v * dt;

        // Store
        profile.jerks[i] = j;
        profile.accelerations[i] = a;
        profile.velocities[i] = v;
        profile.positions[i] = x;
    }

    return profile;
}
