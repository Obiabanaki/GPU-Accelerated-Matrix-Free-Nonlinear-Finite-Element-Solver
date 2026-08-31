/// @file NeoHookeanMaterial.cpp
/// @brief Implementation of NeoHookeanMaterial.
#include "fem/material/NeoHookeanMaterial.hpp"

namespace fem {

NeoHookeanMaterial::NeoHookeanMaterial(double mu, double kappa)
    : mu_(mu), kappa_(kappa) {}

Eigen::Matrix3d NeoHookeanMaterial::computeStress(const Eigen::Matrix3d& F) const {
    // TODO(Phase 0-pre): S = mu*(I - C^-1) + kappa*ln(J)*C^-1, with
    // C = F^T*F, J = det(F). Test against the analytical uniaxial
    // solution before touching anything else in the codebase.
    (void)F;
    return Eigen::Matrix3d::Zero();
}

Eigen::Matrix<double, 6, 6> NeoHookeanMaterial::computeTangent(const Eigen::Matrix3d& F) const {
    // TODO(Phase 0-pre): dS/dE in Voigt notation. MANDATORY: verify against
    // a finite-difference perturbation of computeStress before this is
    // considered done — see project plan's non-negotiable FD check.
    (void)F;
    return Eigen::Matrix<double, 6, 6>::Zero();
}

double NeoHookeanMaterial::strainEnergy(const Eigen::Matrix3d& F) const {
    // TODO(Phase 0-pre): Psi = mu/2*(I1 - 3 - 2*ln(J)) + kappa/2*(ln J)^2
    // (one common compressible Neo-Hookean form — confirm convention
    // against computeStress/computeTangent above before implementing).
    (void)F;
    return 0.0;
}

} // namespace fem
