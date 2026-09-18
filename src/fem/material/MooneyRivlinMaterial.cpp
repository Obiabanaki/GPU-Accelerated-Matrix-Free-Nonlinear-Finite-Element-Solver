/// @file MooneyRivlinMaterial.cpp
/// @brief Implementation of MooneyRivlinMaterial.
/// TRACE MODE — see NeoHookeanMaterial.cpp's file comment for the rationale.
#include "fem/material/MooneyRivlinMaterial.hpp"
#include <iostream>

namespace fem {

MooneyRivlinMaterial::MooneyRivlinMaterial(double c10, double c01, double kappa)
    : c10_(c10), c01_(c01), kappa_(kappa) {
    std::cout << "[MooneyRivlinMaterial] constructed (c10=" << c10_ << ", c01=" << c01_
              << ", kappa=" << kappa_ << ")\n";
}

Eigen::Matrix3d MooneyRivlinMaterial::computeStress(const Eigen::Matrix3d& F) const {
    std::cout << "[MooneyRivlinMaterial::computeStress] would compute S from "
              << "Psi = c10*(I1-3) + c01*(I2-3) + kappa/2*(lnJ)^2 (trace mode)\n";
    (void)F;
    return Eigen::Matrix3d::Zero();
}

Eigen::Matrix<double, 6, 6> MooneyRivlinMaterial::computeTangent(const Eigen::Matrix3d& F) const {
    std::cout << "[MooneyRivlinMaterial::computeTangent] would compute the 6x6 "
              << "material tangent (trace mode)\n";
    (void)F;
    return Eigen::Matrix<double, 6, 6>::Zero();
}

double MooneyRivlinMaterial::strainEnergy(const Eigen::Matrix3d& F) const {
    std::cout << "[MooneyRivlinMaterial::strainEnergy] would compute Psi(F) (trace mode)\n";
    (void)F;
    return 0.0;
}

} // namespace fem
