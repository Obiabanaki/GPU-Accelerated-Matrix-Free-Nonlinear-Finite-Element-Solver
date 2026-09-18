/// @file test_element.cpp
/// @brief Element unit tests.
///
/// TRACE MODE: computeResidual/computeTangentStiffness are placeholders
/// (see Hex8Element.cpp's file comment). Shape functions and the Gauss
/// rule are real (pure geometry), so those get real tests; the physics
/// methods get shape/no-throw checks plus a skipped reminder of the FD
/// check that's mandatory before this element leaves trace mode.
#include <gtest/gtest.h>
#include "fem/element/Hex8Element.hpp"
#include "fem/material/NeoHookeanMaterial.hpp"

namespace {

fem::Hex8Element makeUnitCube() {
    std::array<int, 8> ids = {0, 1, 2, 3, 4, 5, 6, 7};
    std::array<Eigen::Vector3d, 8> coords = {{
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    }};
    return fem::Hex8Element(ids, coords);
}

} // namespace

TEST(Hex8Element, HasStandardTwoPointGaussRule) {
    fem::Hex8Element elem = makeUnitCube();
    EXPECT_EQ(elem.gaussPoints().size(), 8u);
    double totalWeight = 0.0;
    for (const auto& gp : elem.gaussPoints()) totalWeight += gp.weight;
    EXPECT_NEAR(totalWeight, 8.0, 1e-12); // 2x2x2 rule over [-1,1]^3: weight 1 each
}

TEST(Hex8Element, ShapeFunctionsSumToOneEverywhere) {
    // Partition-of-unity property — true regardless of trace mode, since
    // shapeFunctions() is real geometry, not physics.
    fem::Hex8Element elem = makeUnitCube();
    Eigen::VectorXd N;
    Eigen::MatrixXd dN_dxi;
    for (auto xi : {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.5, -0.3, 0.2)}) {
        elem.shapeFunctions(xi, N, dN_dxi);
        EXPECT_NEAR(N.sum(), 1.0, 1e-12);
    }
}

TEST(Hex8Element, PlaceholderResidualHasCorrectSize) {
    fem::Hex8Element elem = makeUnitCube();
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    Eigen::VectorXd R = elem.computeResidual(Eigen::VectorXd::Zero(24), mat);
    EXPECT_EQ(R.size(), 24);
}

TEST(Hex8Element, PlaceholderTangentHasCorrectSize) {
    fem::Hex8Element elem = makeUnitCube();
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    Eigen::MatrixXd K = elem.computeTangentStiffness(Eigen::VectorXd::Zero(24), mat);
    EXPECT_EQ(K.rows(), 24);
    EXPECT_EQ(K.cols(), 24);
}

TEST(Hex8Element, PatchTestReproducesAnalyticalStressState) {
    GTEST_SKIP() << "TODO: real computeResidual required. Not applicable while in trace mode.";
}

TEST(Hex8Element, TangentStiffnessMatchesFiniteDifferenceOfResidual) {
    GTEST_SKIP() << "MANDATORY before this element leaves trace mode: K_ij ~ "
                     "(R_i(u+eps*e_j)-R_i(u))/eps. See ARCHITECTURE.md's design check.";
}
