/// @file test_material.cpp
/// @brief Material unit tests. Step 0-pre acceptance criteria: stress
/// matches the analytical uniaxial solution AND the tangent passes a
/// finite-difference check. Both are required — see project plan.
#include <gtest/gtest.h>
#include "fem/material/NeoHookeanMaterial.hpp"

TEST(NeoHookeanMaterial, IdentityDeformationGivesZeroStress) {
    fem::NeoHookeanMaterial mat(/*mu=*/1.0, /*kappa=*/10.0);
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S = mat.computeStress(F);
    EXPECT_NEAR(S.norm(), 0.0, 1e-8);
}

TEST(NeoHookeanMaterial, MatchesAnalyticalUniaxialTension) {
    // TODO(Phase 0-pre): sweep stretch ratios 0.5-3.0, compare against the
    // closed-form uniaxial Neo-Hookean solution to 1e-8, per plan.
    GTEST_SKIP() << "TODO: implement once computeStress is filled in";
}

TEST(NeoHookeanMaterial, TangentMatchesFiniteDifference) {
    // TODO(Phase 0-pre): MANDATORY per project plan. Perturb F
    // component-wise, recompute S, compare numerical dS/dF (Voigt) against
    // computeTangent to O(h) or O(h^2) truncation-error tolerance.
    GTEST_SKIP() << "TODO: implement once computeTangent is filled in";
}
