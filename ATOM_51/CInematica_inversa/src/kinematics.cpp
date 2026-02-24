#include "kinematics.hpp"

namespace IK {

    double shinLength = 85;
    double thighLength = 85;
    double bodyWidth   = 70.0 / 2.0;
    double bodyLength  = 144.0 / 2.0;

    std::array<double, 3> kinematics_array(
        int leg,
        double xIn, double yIn, double zIn,
        double roll, double pitch, double yawIn
    )
    {
        double hipOffset = 33.0;

        double x = xIn;
        double y = yIn;
        double z = zIn;
        double yaw = yawIn;

        // --- YAW ---
        double yawAngle = yaw * M_PI / 180.0;

        if (leg == 1) { y -= (bodyWidth + hipOffset); x -= bodyLength; }
        else if (leg == 2) { y += (bodyWidth + hipOffset); x -= bodyLength; }
        else if (leg == 3) { y -= (bodyWidth + hipOffset); x += bodyLength; }
        else if (leg == 4) { y += (bodyWidth + hipOffset); x += bodyLength; }

        double existingAngle = atan(y / x);
        double radius = y / sin(existingAngle);
        double demandYaw = existingAngle + yawAngle;

        double xx3 = radius * cos(demandYaw);
        double yy3 = radius * sin(demandYaw);

        if (leg == 1) { yy3 += (bodyWidth + hipOffset); xx3 += bodyLength; }
        else if (leg == 2) { yy3 -= (bodyWidth + hipOffset); xx3 += bodyLength; }
        else if (leg == 3) { yy3 += (bodyWidth + hipOffset); xx3 -= bodyLength; }
        else if (leg == 4) { yy3 -= (bodyWidth + hipOffset); xx3 -= bodyLength; }

        // --- PITCH ---
        if (leg == 1 || leg == 2) {
            pitch *= -1;
            xx3 *= -1;
        }

        double pitchAngle = pitch * M_PI / 180.0;

        double legDiffPitch = sin(pitchAngle) * bodyLength;
        double bodyDiffPitch = cos(pitchAngle) * bodyLength;

        legDiffPitch = z - legDiffPitch;
        double footDisplacementPitch =
            ((bodyDiffPitch - bodyLength) * -1) + xx3;

        double footDisplacementAnglePitch =
            atan(footDisplacementPitch / legDiffPitch);

        double zz2a = legDiffPitch / cos(footDisplacementAnglePitch);
        double footWholeAnglePitch =
            footDisplacementAnglePitch + pitchAngle;

        double zz2 = cos(footWholeAnglePitch) * zz2a;
        double xx1 = sin(footWholeAnglePitch) * zz2a;

        if (leg == 1 || leg == 2) xx1 *= -1;

        // --- ROLL ---
        if (leg == 2 || leg == 3) {
            roll *= -1;
            yy3 *= -1;
        }

        double rollAngle = roll * M_PI / 180.0;

        double legDiffRoll = sin(rollAngle) * bodyWidth;
        double bodyDiffRoll = cos(rollAngle) * bodyWidth;

        legDiffRoll = zz2 - legDiffRoll;

        double footDisplacementRoll =
            (((bodyDiffRoll - bodyWidth) * -1) + hipOffset) - yy3;

        double footDisplacementAngleRoll =
            atan(footDisplacementRoll / legDiffRoll);

        double zz1a = legDiffRoll / cos(footDisplacementAngleRoll);
        double footWholeAngleRoll =
            footDisplacementAngleRoll + rollAngle;

        double zz1 = cos(footWholeAngleRoll) * zz1a;
        double yy1 = sin(footWholeAngleRoll) * zz1a - hipOffset;

        // --- HIP ---
        if (leg == 1 || leg == 4) {
            hipOffset *= -1;
            yy1 *= -1;
        }

        yy1 += hipOffset;

        double hipAngle1a = atan(yy1 / zz1);
        double hipHyp = zz1 / cos(hipAngle1a);

        double hipAngle1b = asin(hipOffset / hipHyp);
        double hipAngle =
            (M_PI - M_PI/2 - hipAngle1b) + hipAngle1a;

        hipAngle -= 1.5708;

        double hipAngleDeg = hipAngle * 180.0 / M_PI;

        // --- SHOULDER ---
        double z2 = hipOffset / tan(hipAngle1b);

        double shoulderAngle2 = atan(xx1 / z2);
        double shoulderAngle2Deg = shoulderAngle2 * 180.0 / M_PI;

        // --- KNEE ---
        double z3 = z2 / cos(shoulderAngle2);

        double shoulderAngle1c =
            (pow(thighLength,2) + pow(z3,2) - pow(shinLength,2)) /
            (2 * thighLength * z3);

        double shoulderAngle1 = acos(shoulderAngle1c);
        double kneeAngle = M_PI - 2 * shoulderAngle1;

        double shoulderAngle1Deg = shoulderAngle1 * 180.0 / M_PI;
        double kneeAngleDeg = kneeAngle * 180.0 / M_PI;

        double shoulderTotalDeg;
        if (leg == 1 || leg == 2)
            shoulderTotalDeg = (shoulderAngle1Deg - 45) + shoulderAngle2Deg;
        else
            shoulderTotalDeg = (shoulderAngle1Deg - 45) - shoulderAngle2Deg;

        // === Correcciones de Sebas ===
        if (leg == 1) {
            hipAngleDeg = round(hipAngleDeg + 120.1);
            shoulderTotalDeg += 75;
        }
        else if (leg == 2) {
            hipAngleDeg += 120.1;
            shoulderTotalDeg = -shoulderTotalDeg + 165;
            kneeAngleDeg = -kneeAngleDeg + 250.75;
        }
        else if (leg == 3) {
            hipAngleDeg = -hipAngleDeg + 120.1;
            shoulderTotalDeg = -shoulderTotalDeg + 165;
            kneeAngleDeg = -kneeAngleDeg + 250.75;
        }
        else if (leg == 4) {
            hipAngleDeg = -hipAngleDeg + 120.1;
            shoulderTotalDeg += 75;
        }

        return {hipAngleDeg, shoulderTotalDeg, kneeAngleDeg};
    }

}
