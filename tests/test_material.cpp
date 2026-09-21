/// @file test_material.cpp
/// @brief Material unit tests.
///
/// NeoHookeanMaterial and MooneyRivlinMaterial are both real, closed-form
/// hyperelastic models now (see their .cpp file comments for the derived
/// stress/tangent formulas and documented limitations). Both get the same
/// generic checks below via checkStressAndTangentConsistency(): energy
/// consistency (S == dPsi/dE) and tangent consistency (dS/dE ==
/// computeTangent()), each verified by a central-difference perturbation
/// of F — the "mandatory FD check" required for any numerical
/// formulation — plus a known-solution check at the undeformed state and
/// a documented-limitation check for non-positive det(F).
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "fem/material/NeoHookeanMaterial.hpp"
#include "fem/material/MooneyRivlinMaterial.hpp"

namespace {

// A handful of deformation states (identity, pure stretch, combined
// stretch+shear) used to exercise the FD checks away from the trivial
// F=I case as well as at it.
std::vector<Eigen::Matrix3d> sampleDeformations() {
    Eigen::Matrix3d stretch;
    stretch << 1.2, 0.0, 0.0,
               0.0, 0.9, 0.0,
               0.0, 0.0, 1.05;
    Eigen::Matrix3d stretchAndShear;
    stretchAndShear << 1.1, 0.1, -0.05,
                       0.0, 0.95, 0.1,
                       0.05, 0.0, 1.0;
    return {Eigen::Matrix3d::Identity(), stretch, stretchAndShear};
}

// Voigt conventions matching Hex8Element.cpp/VoigtUtil.hpp: S_voigt uses
// plain tensor components, E_voigt uses the engineering (2x shear) factor
// so that S_voigt = Cvoigt * E_voigt reproduces the tensor contraction.
Eigen::Matrix<double, 6, 1> stressVoigt(const Eigen::Matrix3d& S) {
    Eigen::Matrix<double, 6, 1> v;
    v << S(0, 0), S(1, 1), S(2, 2), S(0, 1), S(1, 2), S(0, 2);
    return v;
}
Eigen::Matrix<double, 6, 1> engineeringStrainVoigt(const Eigen::Matrix3d& E) {
    Eigen::Matrix<double, 6, 1> v;
    v << E(0, 0), E(1, 1), E(2, 2), 2 * E(0, 1), 2 * E(1, 2), 2 * E(0, 2);
    return v;
}

// Mandatory FD check, generic over any Material: at base state F0, probe
// each of the 9 entries of F one at a time (dF = e_i outer e_J) and verify
//   (a) energy consistency: dPsi/dF_iJ (central difference) matches the
//       analytically-predicted S:dE (S = computeStress(F0), dE the exact
//       linearization of E=(F^T F-I)/2 for this dF), and
//   (b) tangent consistency: dS/dF_iJ (central difference) matches
//       computeTangent(F0) contracted with the same dE.
// dE is exact (not itself an approximation) because E is quadratic in F;
// only the resulting central differences in S/Psi are approximate.
void checkStressAndTangentConsistency(const fem::Material& material,
                                       const Eigen::Matrix3d& F0,
                                       double eps = 1e-6) {
    const Eigen::Matrix3d S0 = material.computeStress(F0);
    const Eigen::Matrix<double, 6, 6> C = material.computeTangent(F0);

    for (int i = 0; i < 3; ++i) {      // loop over the perturbed dF's row
        for (int J = 0; J < 3; ++J) {  // loop over the perturbed dF's column
            Eigen::Matrix3d dF = Eigen::Matrix3d::Zero();
            dF(i, J) = 1.0;
            const Eigen::Matrix3d Fplus = F0 + eps * dF;
            const Eigen::Matrix3d Fminus = F0 - eps * dF;

            const double psiFd = (material.strainEnergy(Fplus) - material.strainEnergy(Fminus)) / (2 * eps);
            const Eigen::Matrix3d Sfd =
                (material.computeStress(Fplus) - material.computeStress(Fminus)) / (2 * eps);

            const Eigen::Matrix3d dE = 0.5 * (dF.transpose() * F0 + F0.transpose() * dF);
            const double psiPredicted = (S0.array() * dE.array()).sum(); // S : dE
            const Eigen::Matrix<double, 6, 1> SfdPredicted = C * engineeringStrainVoigt(dE);

            EXPECT_NEAR(psiFd, psiPredicted, 1e-5) << "F0 row " << i << " col " << J;
            EXPECT_LT((stressVoigt(Sfd) - SfdPredicted).norm(), 1e-4) << "F0 row " << i << " col " << J;
        }
    }
}

} // namespace

// ---------------------------------------------------------------------
// NeoHookeanMaterial
// ---------------------------------------------------------------------

TEST(NeoHookeanMaterial, ConstructsWithoutThrowing) {
    EXPECT_NO_THROW(fem::NeoHookeanMaterial(1.0, 10.0));
}

TEST(NeoHookeanMaterial, ComputeStressAndTangentHaveCorrectShape) {
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    const Eigen::Matrix3d S = mat.computeStress(Eigen::Matrix3d::Identity());
    const auto C = mat.computeTangent(Eigen::Matrix3d::Identity());
    EXPECT_EQ(S.rows(), 3);
    EXPECT_EQ(S.cols(), 3);
    EXPECT_EQ(C.rows(), 6);
    EXPECT_EQ(C.cols(), 6);
}

TEST(NeoHookeanMaterial, IsStressAndEnergyFreeAtReferenceConfiguration) {
    // Known solution: F=I => C=Cinv=I, ln(J)=0, so S = mu*(I-I) + 0 = 0
    // and Psi = 0 identically, for any mu/kappa.
    fem::NeoHookeanMaterial mat(3.0, 15.0);
    EXPECT_LT(mat.computeStress(Eigen::Matrix3d::Identity()).norm(), 1e-12);
    EXPECT_NEAR(mat.strainEnergy(Eigen::Matrix3d::Identity()), 0.0, 1e-12);
}

TEST(NeoHookeanMaterial, StressAndTangentAreConsistentWithEnergyByFiniteDifference) {
    fem::NeoHookeanMaterial mat(3.0, 15.0);
    for (const Eigen::Matrix3d& F0 : sampleDeformations()) checkStressAndTangentConsistency(mat, F0);
}

TEST(NeoHookeanMaterial, NonFiniteForNonPositiveJacobian) {
    // ln(J) is undefined for det(F)<=0; NeoHookeanMaterial does not guard
    // against inverted/degenerate F (documented limitation in the .cpp
    // file comment), so this locks in the current behavior (non-finite
    // stress) instead of a crash.
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(2, 2) = -1.0; // det(F) = -1
    EXPECT_FALSE(mat.computeStress(F).allFinite());
}

// ---------------------------------------------------------------------
// MooneyRivlinMaterial
// ---------------------------------------------------------------------

TEST(MooneyRivlinMaterial, ConstructsWithoutThrowing) {
    EXPECT_NO_THROW(fem::MooneyRivlinMaterial(1.0, 0.5, 10.0));
}

TEST(MooneyRivlinMaterial, ComputeStressAndTangentHaveCorrectShape) {
    fem::MooneyRivlinMaterial mat(1.0, 0.5, 10.0);
    const Eigen::Matrix3d S = mat.computeStress(Eigen::Matrix3d::Identity());
    const auto C = mat.computeTangent(Eigen::Matrix3d::Identity());
    EXPECT_EQ(S.rows(), 3);
    EXPECT_EQ(S.cols(), 3);
    EXPECT_EQ(C.rows(), 6);
    EXPECT_EQ(C.cols(), 6);
}

TEST(MooneyRivlinMaterial, IsEnergyFreeAtReferenceConfiguration) {
    // Known solution: F=I => I1=I2=3, ln(J)=0, so Psi = c10*0+c01*0+0 = 0
    // for any c10/c01/kappa. (Stress at F=I is generally nonzero for this
    // raw-invariant 2-parameter model — see the .cpp file comment's
    // documented limitation — so only the energy is checked exactly here.)
    fem::MooneyRivlinMaterial mat(2.0, 1.0, 15.0);
    EXPECT_NEAR(mat.strainEnergy(Eigen::Matrix3d::Identity()), 0.0, 1e-12);
}

TEST(MooneyRivlinMaterial, StressAndTangentAreConsistentWithEnergyByFiniteDifference) {
    fem::MooneyRivlinMaterial mat(2.0, 1.0, 15.0);
    for (const Eigen::Matrix3d& F0 : sampleDeformations()) checkStressAndTangentConsistency(mat, F0);
}

TEST(MooneyRivlinMaterial, NonFiniteForNonPositiveJacobian) {
    // Same documented limitation as NeoHookeanMaterial: ln(J) undefined
    // for det(F)<=0, not guarded against here either.
    fem::MooneyRivlinMaterial mat(2.0, 1.0, 15.0);
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(2, 2) = -1.0;
    EXPECT_FALSE(mat.computeStress(F).allFinite());
}

