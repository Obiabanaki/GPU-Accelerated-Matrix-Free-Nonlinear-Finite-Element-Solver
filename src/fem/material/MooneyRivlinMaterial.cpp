/// @file MooneyRivlinMaterial.cpp
/// @brief Implementation of MooneyRivlinMaterial: two-parameter
/// compressible Mooney-Rivlin hyperelasticity.
///
/// Formulation (all quantities below a function of the deformation
/// gradient F passed into each method): let C = F^T*F, Cinv = C^-1,
/// I1 = tr(C), I2 = (1/2)*(I1^2 - tr(C*C)) (second invariant of C),
/// J = det(F).
///
///   Psi(F) = c10*(I1 - 3) + c01*(I2 - 3) + (kappa/2)*(ln J)^2
///
/// Differentiating twice (S = 2*dPsi/dC, dS/dE = 4*d2Psi/dC2) gives the
/// closed forms implemented below:
///
///   S_IJ = 2*(c10 + c01*I1)*I_IJ - 2*c01*C_IJ + kappa*ln(J)*Cinv_IJ
///
///   dS_IJ/dE_KL = 4*c01*(I_IJ*I_KL - Isym_IJKL)
///                 + kappa*Cinv_IJ*Cinv_KL
///                 - kappa*ln(J)*(Cinv_IK*Cinv_JL + Cinv_IL*Cinv_JK)
///
/// where Isym_IJKL = (1/2)*(delta_IK*delta_JL + delta_IL*delta_JK) is the
/// symmetric 4th-order identity built from the 3x3 identity I. See
/// NeoHookeanMaterial.cpp's file comment for the shared Voigt/engineering-
/// convention bookkeeping. Both formulas are FD-verified against
/// strainEnergy() in tests/test_material.cpp.
///
/// Limitations:
///  - This uses the raw (non-isochoric) invariants I1/I2 rather than a
///    volumetric/isochoric split, so — unlike NeoHookeanMaterial — S is
///    not guaranteed to vanish at the undeformed state F=I unless
///    c10/c01/kappa happen to satisfy a particular relation. Psi(I) = 0
///    always holds (verified in tests/test_material.cpp), but
///    stress-at-rest does not; this is a known simplification of the
///    2-parameter model as specified, not a bug.
///  - Same det(F) > 0 assumption/limitation as NeoHookeanMaterial (ln(J)
///    is undefined for non-positive J; not guarded against here either).
#include "fem/material/MooneyRivlinMaterial.hpp"
#include "fem/internal/VoigtUtil.hpp"
#include <cmath>

namespace fem {

MooneyRivlinMaterial::MooneyRivlinMaterial(double c10, double c01, double kappa)
    : c10_(c10), c01_(c01), kappa_(kappa) {}

Eigen::Matrix3d MooneyRivlinMaterial::computeStress(const Eigen::Matrix3d& F) const {
    const Eigen::Matrix3d C = F.transpose() * F;
    const Eigen::Matrix3d Cinv = C.inverse();
    const double I1 = C.trace();
    const double lnJ = std::log(F.determinant());
    return 2.0 * (c10_ + c01_ * I1) * Eigen::Matrix3d::Identity() - 2.0 * c01_ * C
         + kappa_ * lnJ * Cinv;
}

Eigen::Matrix<double, 6, 6> MooneyRivlinMaterial::computeTangent(const Eigen::Matrix3d& F) const {
    const Eigen::Matrix3d C = F.transpose() * F;
    const Eigen::Matrix3d Cinv = C.inverse();
    const double lnJ = std::log(F.determinant());
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    // Builds C_IJKL index-by-index (I3 supplies the delta_IJ terms) and
    // flattens it via VoigtUtil's engineering-convention helper.
    return internal::buildEngineeringVoigtTangent([&](int I, int J, int K, int L) {
        const double isym = 0.5 * (I3(I, K) * I3(J, L) + I3(I, L) * I3(J, K));
        return 4.0 * c01_ * (I3(I, J) * I3(K, L) - isym)
             + kappa_ * Cinv(I, J) * Cinv(K, L)
             - kappa_ * lnJ * (Cinv(I, K) * Cinv(J, L) + Cinv(I, L) * Cinv(J, K));
    });
}

double MooneyRivlinMaterial::strainEnergy(const Eigen::Matrix3d& F) const {
    const Eigen::Matrix3d C = F.transpose() * F;
    const double I1 = C.trace();
    const double I2 = 0.5 * (I1 * I1 - (C * C).trace());
    const double lnJ = std::log(F.determinant());
    return c10_ * (I1 - 3.0) + c01_ * (I2 - 3.0) + 0.5 * kappa_ * lnJ * lnJ;
}

} // namespace fem
