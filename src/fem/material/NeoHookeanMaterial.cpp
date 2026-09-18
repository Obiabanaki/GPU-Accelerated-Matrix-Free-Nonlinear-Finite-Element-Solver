/// @file NeoHookeanMaterial.cpp
/// @brief Implementation of NeoHookeanMaterial.
///
/// TRACE MODE: this class's constructor is real (stores parameters), but
/// computeStress/computeTangent/strainEnergy only print what they would
/// compute and return placeholder zero values. The full compressible
/// Neo-Hookean formulas (S = mu(I-Cinv) + kappa*lnJ*Cinv, and the matching
/// material tangent) were implemented and FD-verified in an earlier pass
/// of this project — ask to have them merged back in once the pipeline
/// wiring below is validated and it's time to move past the facade stage.
#include "fem/material/NeoHookeanMaterial.hpp"
#include <iostream>

namespace fem {

NeoHookeanMaterial::NeoHookeanMaterial(double mu, double kappa)
    : mu_(mu), kappa_(kappa) {
    std::cout << "[NeoHookeanMaterial] constructed (mu=" << mu_ << ", kappa=" << kappa_ << ")\n";
}

Eigen::Matrix3d NeoHookeanMaterial::computeStress(const Eigen::Matrix3d& F) const {
    std::cout << "[NeoHookeanMaterial::computeStress] would compute S = mu*(I - Cinv) "
              << "+ kappa*ln(J)*Cinv for F with det(F)=" << F.determinant() << " (trace mode)\n";
    return Eigen::Matrix3d::Zero();
}

Eigen::Matrix<double, 6, 6> NeoHookeanMaterial::computeTangent(const Eigen::Matrix3d& F) const {
    std::cout << "[NeoHookeanMaterial::computeTangent] would compute the 6x6 material "
              << "tangent for F with det(F)=" << F.determinant() << " (trace mode)\n";
    return Eigen::Matrix<double, 6, 6>::Zero();
}

double NeoHookeanMaterial::strainEnergy(const Eigen::Matrix3d& F) const {
    std::cout << "[NeoHookeanMaterial::strainEnergy] would compute Psi(F) (trace mode)\n";
    (void)F;
    return 0.0;
}

} // namespace fem
