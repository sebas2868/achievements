#pragma once
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

// ====================================================
// 📐 Declaraciones de funciones auxiliares
// ====================================================

// Convierte dos puntos cartesianos (p1, p2) a ángulo polar (0 - 2π)
double point_to_rad(double p1, double p2);

// Genera una matriz de rotación 3D a partir de roll, pitch, yaw
Matrix3d RotMatrix3D(Vector3d rotation, bool is_radians = true, const std::string& order = "xyz");
