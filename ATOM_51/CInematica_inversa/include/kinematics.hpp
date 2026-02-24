#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include <array>
#include <cmath>

namespace IK {

    // --- Parámetros del robot ---
    extern double shinLength;
    extern double thighLength;
    extern double bodyWidth;
    extern double bodyLength;

    // --- Prototipo de función ---
    std::array<double, 3> kinematics_array(
        int leg,
        double xIn, double yIn, double zIn,
        double roll, double pitch, double yawIn
    );

}

#endif

