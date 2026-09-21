/// @file NeoHookeanMaterial.cpp
/// @brief Implementation of NeoHookeanMaterial: compressible Neo-Hookean
/// hyperelasticity.
///
/// Formulation (all quantities below a function of the deformation
/// gradient F passed into each method): let C = F^T*F (right Cauchy-Green
/// tensor), Cinv = C^-1, I1 = tr(C), J = det(F).
///
///   Psi(F) = (mu/2)*(I1 - 3) - mu*ln(J) + (kappa/2)*(ln J)^2
///
/// Differentiating twice (S = 2*dPsi/dC, dS/dE = 4*d2Psi/dC2, using
/// E = (C-I)/2) gives the closed forms implemented below:
///
///   S_IJ        = mu*(I_IJ - Cinv_IJ) + kappa*ln(J)*Cinv_IJ
///   dS_IJ/dE_KL = kappa*Cinv_IJ*Cinv_KL
///                 + (mu - kappa*ln(J)) * (Cinv_IK*Cinv_JL + Cinv_IL*Cinv_JK)
///
/// This is the standard compressible Neo-Hookean model (e.g. Bonet &
/// Wood), with kappa playing the role of the volumetric/near-
/// incompressibility parameter and mu the shear modulus. Both formulas
/// are FD-verified against strainEnergy() in tests/test_material.cpp
/// (S == dPsi/dE and the tangent == dS/dE, both checked by perturbing F).
///
/// Limitation: this model assumes det(F) > 0 (a non-inverted, non-
/// degenerate element, so ln(J) is finite); like Hex8Element, it does not
/// itself guard against or reject non-positive J — see
/// tests/test_material.cpp's NonFiniteForNonPositiveJacobian test, which
/// locks in the resulting non-finite output as the current, accepted
/// behavior rather than a crash.
#include "fem/material/NeoHookeanMaterial.hpp"
#include "fem/internal/VoigtUtil.hpp"
#include <cmath>

namespace fem {

NeoHookeanMaterial::NeoHookeanMaterial(double mu, double kappa) : mu_(mu), kappa_(kappa) {}

Eigen::Matrix3d NeoHookeanMaterial::computeStress(const Eigen::Matrix3d& F) const {
    const Eigen::Matrix3d Cinv = (F.transpose() * F).inverse();
    const double lnJ = std::log(F.determinant());
    return mu_ * (Eigen::Matrix3d::Identity() - Cinv) + kappa_ * lnJ * Cinv;
}

Eigen::Matrix<double, 6, 6> NeoHookeanMaterial::computeTangent(const Eigen::Matrix3d& F) const {
    const Eigen::Matrix3d Cinv = (F.transpose() * F).inverse();
    const double lnJ = std::log(F.determinant());
    const double muEff = mu_ - kappa_ * lnJ; // coefficient of the symmetric Cinv-Cinv product below

    // Builds C_IJKL index-by-index and flattens it via VoigtUtil's
    // engineering-convention helper (see that header's comment); the
    // lambda captures Cinv/muEff by reference since it only lives for the
    // duration of this call.

    // internal is a namespace defined in VoigtUtil.hpp that contains the
    // [] is for lambada function, [&] for lamvada function which can use variable in surrounding scope.
    return internal::buildEngineeringVoigtTangent([&](int I, int J, int K, int L) {
        return kappa_ * Cinv(I, J) * Cinv(K, L)
             + muEff * (Cinv(I, K) * Cinv(J, L) + Cinv(I, L) * Cinv(J, K));
    });
}

double NeoHookeanMaterial::strainEnergy(const Eigen::Matrix3d& F) const {
    const double I1 = (F.transpose() * F).trace();
    const double lnJ = std::log(F.determinant());
    return 0.5 * mu_ * (I1 - 3.0) - mu_ * lnJ + 0.5 * kappa_ * lnJ * lnJ;
}

} // namespace fem
