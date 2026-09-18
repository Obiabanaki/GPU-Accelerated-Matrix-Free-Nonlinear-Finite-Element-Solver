/// @file test_material.cpp
/// @brief Material unit tests.
///
/// TRACE MODE: computeStress/computeTangent/strainEnergy are placeholders
/// (see NeoHookeanMaterial.cpp's file comment), so the tests below check
/// what's actually true right now — construction succeeds and placeholder
/// values have the right shape. The mandatory FD tangent check from the
/// project plan is written but skipped until real math is merged back in;
/// deleting it would lose the reminder that it's still required before
/// this material is considered "done."
#include <gtest/gtest.h>
#include "fem/material/NeoHookeanMaterial.hpp"

TEST(NeoHookeanMaterial, ConstructsWithoutThrowing) {
    EXPECT_NO_THROW(fem::NeoHookeanMaterial(1.0, 10.0));
}

TEST(NeoHookeanMaterial, PlaceholderStressHasCorrectShape) {
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S = mat.computeStress(F);
    EXPECT_EQ(S.rows(), 3);
    EXPECT_EQ(S.cols(), 3);
}

TEST(NeoHookeanMaterial, PlaceholderTangentHasCorrectShape) {
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    auto C = mat.computeTangent(F);
    EXPECT_EQ(C.rows(), 6);
    EXPECT_EQ(C.cols(), 6);
}

TEST(NeoHookeanMaterial, MatchesAnalyticalUniaxialTension) {
    GTEST_SKIP() << "TODO: real computeStress required — see project plan's "
                     "Phase 0-pre acceptance criteria. Not applicable while in trace mode.";
}

TEST(NeoHookeanMaterial, TangentMatchesFiniteDifference) {
    GTEST_SKIP() << "MANDATORY before this material leaves trace mode: perturb F "
                     "component-wise, confirm numerical dS/dF matches computeTangent. "
                     "See project plan's non-negotiable FD check.";
}
