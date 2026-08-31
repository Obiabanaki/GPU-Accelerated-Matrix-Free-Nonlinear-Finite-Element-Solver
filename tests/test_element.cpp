/// @file test_element.cpp
/// @brief Element unit tests, including the tangent-stiffness FD check
/// flagged as the single most valuable test in ARCHITECTURE.md.
#include <gtest/gtest.h>
#include "fem/element/Hex8Element.hpp"
#include "fem/material/NeoHookeanMaterial.hpp"

TEST(Hex8Element, PatchTestReproducesAnalyticalStressState) {
    // TODO(Phase 0-pre): single element under prescribed uniform
    // deformation; compare against the analytical patch-test stress state.
    GTEST_SKIP() << "TODO: implement once Hex8Element is filled in";
}

TEST(Hex8Element, TangentStiffnessMatchesFiniteDifferenceOfResidual) {
    // TODO(Phase 0-pre/1.2): K_ij ~ (R_i(u+eps*e_j) - R_i(u)) / eps.
    // Catches both missing geometric-stiffness terms and sign errors.
    GTEST_SKIP() << "TODO: implement once computeTangentStiffness is filled in";
}
