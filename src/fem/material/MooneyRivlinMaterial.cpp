/// @file MooneyRivlinMaterial.cpp
/// @brief Implementation of MooneyRivlinMaterial.
#include "fem/material/MooneyRivlinMaterial.hpp"

namespace fem {

MooneyRivlinMaterial::MooneyRivlinMaterial(double c10, double c01, double kappa)
    : c10_(c10), c01_(c01), kappa_(kappa) {}

Eigen::Matrix3d MooneyRivlinMaterial::computeStress(const Eigen::Matrix3d& F) const {
    // TODO(Step 1.5): S from Psi = c10*(I1-3) + c01*(I2-3) + kappa/2*(ln J)^2.
    (void)F;
    return Eigen::Matrix3d::Zero();
}

Eigen::Matrix<double, 6, 6> MooneyRivlinMaterial::computeTangent(const Eigen::Matrix3d& F) const {
    // TODO(Step 1.5): FD-verify exactly as NeoHookeanMaterial's tangent was.
    (void)F;
    return Eigen::Matrix<double, 6, 6>::Zero();
}

double MooneyRivlinMaterial::strainEnergy(const Eigen::Matrix3d& F) const {
    (void)F;
    return 0.0;
}

} // namespace fem
