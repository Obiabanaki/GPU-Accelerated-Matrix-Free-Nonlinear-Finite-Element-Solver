/// @file test_element.cpp
/// @brief Element unit tests.
///
/// Hex8Element has a total-Lagrangian formulation
/// (see Hex8Element.cpp), so its residual/tangent are tested :
/// known-solution patch tests, structural invariants (translational
/// invariance, tangent symmetry), and the mandatory finite-difference
/// check that K == dR/du. The production Material subclasses
/// (NeoHookeanMaterial, MooneyRivlinMaterial) are still trace-mode
/// placeholders that always return S=0, which would make those checks
/// vacuous, so a small self-consistent St. Venant-Kirchhoff material is
/// defined locally below purely to exercise Hex8Element's math.
///
/// Tet4Element is still trace mode (see Tet4Element.cpp), so only its real
/// parts (shape functions, node ids, quadrature rule) get real tests; its
/// residual/tangent get placeholder-shape checks plus the same mandatory
/// FD-check reminder Hex8Element used to carry before it left trace mode.
#include <gtest/gtest.h>
#include <cmath>
#include "fem/element/Hex8Element.hpp"
#include "fem/element/Tet4Element.hpp"
#include "fem/material/NeoHookeanMaterial.hpp"

namespace {

/// @brief Self-consistent St. Venant-Kirchhoff material, used only in this
/// test file to exercise Hex8Element's real formulation.
///
/// S = lambda*tr(E)*I + 2*mu*E is linear in the Green-Lagrange strain E, so
/// its material tangent dS/dE is the constant, textbook isotropic
/// elasticity matrix — in the *engineering*-shear Voigt convention (shear
/// rows/cols carry mu, not 2*mu) to match Hex8Element's B, which maps du to
/// engineering-strain increments (see Hex8Element.cpp's file comment).
/// This bypasses NeoHookeanMaterial/MooneyRivlinMaterial, which are still
/// trace-mode placeholders that always return S=0 and would make any
/// residual/tangent check on Hex8Element pass vacuously.
class StVenantKirchhoffMaterial : public fem::Material {
public:
    StVenantKirchhoffMaterial(double lambda, double mu) : lambda_(lambda), mu_(mu) {}

    Eigen::Matrix3d computeStress(const Eigen::Matrix3d& F) const override {
        const Eigen::Matrix3d E = 0.5 * (F.transpose() * F - Eigen::Matrix3d::Identity());
        return lambda_ * E.trace() * Eigen::Matrix3d::Identity() + 2.0 * mu_ * E;
    }

    Eigen::Matrix<double, 6, 6> computeTangent(const Eigen::Matrix3d& /*F*/) const override {
        // Constant because S is linear in E for St. Venant-Kirchhoff.
        Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) C(i, j) = lambda_ + (i == j ? 2.0 * mu_ : 0.0);
        C(3, 3) = C(4, 4) = C(5, 5) = mu_;
        return C;
    }

    double strainEnergy(const Eigen::Matrix3d& F) const override {
        const Eigen::Matrix3d E = 0.5 * (F.transpose() * F - Eigen::Matrix3d::Identity());
        return 0.5 * lambda_ * E.trace() * E.trace() + mu_ * (E * E).trace();
    }

private:
    double lambda_;
    double mu_;
};

// Reference coordinates of the unit-cube Hex8Element built by makeUnitCube(),
// exposed separately so tests can build affine displacement fields u_a =
// (F0 - I) * X0_a from the same reference positions.
const std::array<Eigen::Vector3d, 8>& unitCubeCorners() {
    // static here means the local variable coords is initialized once, 
    // the first time unitCubeCorners() is called, and then persists 
    // (keeps its value) across all subsequent calls — 
    // it isn't re-constructed every time the function runs.
    static const std::array<Eigen::Vector3d, 8> coords = {{
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    }};
    return coords;
}


fem::Hex8Element makeUnitCube() {
    std::array<int, 8> ids = {0, 1, 2, 3, 4, 5, 6, 7};
    return fem::Hex8Element(ids, unitCubeCorners());
}

// A non-cuboid, non-degenerate hexahedron (corners perturbed off-axis), so
// element-level tests aren't accidentally only valid for the axis-aligned
// unit cube's especially simple Jacobian.
fem::Hex8Element makeDistortedHex() {
    std::array<int, 8> ids = {10, 11, 12, 13, 14, 15, 16, 17};
    std::array<Eigen::Vector3d, 8> coords = {{
        {0, 0, 0}, {1.2, -0.1, 0.05}, {1.1, 1.05, -0.05}, {-0.05, 1.1, 0},
        {0.05, 0, 1.0}, {1.0, 0, 1.1}, {1.05, 1.0, 0.95}, {0, 1.0, 1.05},
    }};
    return fem::Hex8Element(ids, coords);
}

// Deterministic small "arbitrary" displacement, used by tests that only
// need a generic, non-symmetric, non-trivial u rather than a specific
// known solution (kept small so the cube/distorted-hex geometry stays
// well-conditioned).
Eigen::VectorXd arbitrarySmallDisplacement() {
    Eigen::VectorXd u(24);
    for (int i = 0; i < 24; ++i) u(i) = 0.02 * std::sin(0.7 * i + 0.3);
    return u;
}

// Central-difference Jacobian of computeResidual w.r.t. u, column by
// column; compared against computeTangentStiffness in the mandatory FD
// check below. Assumes computeResidual is smooth in u at this point (true
// away from degenerate/inverted geometry).
Eigen::MatrixXd finiteDifferenceTangent(const fem::Hex8Element& elem,
                                         const Eigen::VectorXd& u,
                                         const fem::Material& material,
                                         double eps = 1e-6) {
    Eigen::MatrixXd Kfd = Eigen::MatrixXd::Zero(24, 24);
    for (int j = 0; j < 24; ++j) { // loop over the 24 displacement DOFs being perturbed, one column of Kfd each
        Eigen::VectorXd uPlus = u, uMinus = u;
        uPlus(j) += eps;
        uMinus(j) -= eps;
        Kfd.col(j) = (elem.computeResidual(uPlus, material) - elem.computeResidual(uMinus, material)) / (2.0 * eps);
    }
    return Kfd;
}

fem::Tet4Element makeUnitTet() {
    std::array<int, 4> ids = {0, 1, 2, 3};
    std::array<Eigen::Vector3d, 4> coords = {{
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
    }};
    return fem::Tet4Element(ids, coords);
}

} // namespace

// ---------------------------------------------------------------------
// Hex8Element
// ---------------------------------------------------------------------

TEST(Hex8Element, NodeIdsMatchConstructorArgument) {
    fem::Hex8Element elem = makeUnitCube();
    const std::vector<int> expected = {0, 1, 2, 3, 4, 5, 6, 7};
    EXPECT_EQ(elem.nodeIds(), expected);
}

TEST(Hex8Element, HasStandardTwoPointGaussRule) {
    fem::Hex8Element elem = makeUnitCube();
    EXPECT_EQ(elem.gaussPoints().size(), 8u);
    double totalWeight = 0.0;
    for (const auto& gp : elem.gaussPoints()) totalWeight += gp.weight;
    EXPECT_NEAR(totalWeight, 8.0, 1e-12); // 2x2x2 rule over [-1,1]^3: weight 1 each
}

TEST(Hex8Element, ShapeFunctionsSumToOneEverywhere) {
    // Partition-of-unity property, required for rigid-body-mode exactness.
    fem::Hex8Element elem = makeUnitCube();
    Eigen::VectorXd N;
    Eigen::MatrixXd dN_dxi;
    for (auto xi : {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.5, -0.3, 0.2)}) {
        elem.shapeFunctions(xi, N, dN_dxi);
        EXPECT_NEAR(N.sum(), 1.0, 1e-12);
    }
}

TEST(Hex8Element, ShapeFunctionsSatisfyKroneckerDeltaAtCorners) {
    // N_a(xi_b) == delta_ab: each shape function must be 1 at its own
    // parametric corner and 0 at every other corner.
    fem::Hex8Element elem = makeUnitCube();
    const std::array<Eigen::Vector3d, 8> corners = {{
        {-1, -1, -1}, { 1, -1, -1}, { 1,  1, -1}, {-1,  1, -1},
        {-1, -1,  1}, { 1, -1,  1}, { 1,  1,  1}, {-1,  1,  1},
    }};
    Eigen::VectorXd N;
    Eigen::MatrixXd dN_dxi;
    for (int b = 0; b < 8; ++b) { // loop over the corner being evaluated at
        elem.shapeFunctions(corners[b], N, dN_dxi);
        for (int a = 0; a < 8; ++a) EXPECT_NEAR(N(a), (a == b) ? 1.0 : 0.0, 1e-12); // loop over the 8 shape functions
    }
}

TEST(Hex8Element, ShapeFunctionGradientsMatchFiniteDifference) {
    // dN_dxi from shapeFunctions() must match the central-difference
    // derivative of N(xi) taken one parametric direction at a time.
    fem::Hex8Element elem = makeUnitCube();
    const Eigen::Vector3d xi(0.2, -0.4, 0.6);
    Eigen::VectorXd N;
    Eigen::MatrixXd dN_dxi;
    elem.shapeFunctions(xi, N, dN_dxi);

    const double eps = 1e-6;
    for (int dim = 0; dim < 3; ++dim) { // loop over the 3 parametric directions being perturbed
        Eigen::Vector3d xiPlus = xi, xiMinus = xi;
        xiPlus(dim) += eps;
        xiMinus(dim) -= eps;
        Eigen::VectorXd Nplus, Nminus;
        Eigen::MatrixXd dummy;
        elem.shapeFunctions(xiPlus, Nplus, dummy);
        elem.shapeFunctions(xiMinus, Nminus, dummy);
        const Eigen::VectorXd fd = (Nplus - Nminus) / (2.0 * eps);
        for (int a = 0; a < 8; ++a) EXPECT_NEAR(dN_dxi(a, dim), fd(a), 1e-6); // loop over the 8 shape functions
    }
}

TEST(Hex8Element, ComputeResidualHasSizeTwentyFour) {
    fem::Hex8Element elem = makeUnitCube();
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    Eigen::VectorXd R = elem.computeResidual(Eigen::VectorXd::Zero(24), mat);
    EXPECT_EQ(R.size(), 24);
}

TEST(Hex8Element, ComputeTangentStiffnessHasSizeTwentyFourSquared) {
    fem::Hex8Element elem = makeUnitCube();
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    Eigen::MatrixXd K = elem.computeTangentStiffness(Eigen::VectorXd::Zero(24), mat);
    EXPECT_EQ(K.rows(), 24);
    EXPECT_EQ(K.cols(), 24);
}

TEST(Hex8Element, ComputeResidualIsZeroAtZeroStrain) {
    // F=I => Green-Lagrange strain E=0 => S=0 for St. Venant-Kirchhoff, so
    // the residual (integral of B^T*S over the element) must vanish.
    fem::Hex8Element elem = makeUnitCube();
    StVenantKirchhoffMaterial material(5.0, 2.0);
    const Eigen::VectorXd R = elem.computeResidual(Eigen::VectorXd::Zero(24), material);
    EXPECT_LT(R.norm(), 1e-12); // isApprox() against exact zero would require bit-exact equality
}

TEST(Hex8Element, NodalForcesSumToZeroForArbitraryDisplacement) {
    // Internal-force reciprocity: summing R_a over all 8 nodes must vanish
    // for any displacement field and any stress state, because the
    // reference shape-function gradients sum to zero at every Gauss point
    // (gradient of the constant field sum_a(N_a)=1). This holds regardless
    // of the material law, so it exercises assembly/indexing correctness
    // in computeResidual independently of any specific known solution.
    for (fem::Hex8Element elem : {makeUnitCube(), makeDistortedHex()}) { // loop over both test geometries
        StVenantKirchhoffMaterial material(5.0, 2.0);
        const Eigen::VectorXd R = elem.computeResidual(arbitrarySmallDisplacement(), material);
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (int a = 0; a < 8; ++a) sum += R.segment<3>(3 * a); // loop over the 8 nodes' 3-vectors
        // isApprox() against an exact zero vector degenerates to requiring
        // exact equality (Eigen's relative tolerance scales with the
        // right-hand side's norm), so compare the norm directly instead.
        EXPECT_LT(sum.norm(), 1e-9);
    }
}

TEST(Hex8Element, ComputeTangentStiffnessIsSymmetric) {
    // K = B^T*C*B + geometric term is symmetric whenever the material
    // tangent C is symmetric, which holds for any hyperelastic material
    // derived from a strain-energy potential (true here for St.
    // Venant-Kirchhoff).
    for (fem::Hex8Element elem : {makeUnitCube(), makeDistortedHex()}) { // loop over both test geometries
        StVenantKirchhoffMaterial material(5.0, 2.0);
        const Eigen::MatrixXd K = elem.computeTangentStiffness(arbitrarySmallDisplacement(), material);
        EXPECT_TRUE(K.isApprox(K.transpose(), 1e-9));
    }
}

TEST(Hex8Element, TangentStiffnessMatchesFiniteDifferenceOfResidual) {
    // Mandatory check: K_ij ~= (R_i(u+eps*e_j) - R_i(u-eps*e_j)) / (2*eps).
    // Uses the local StVenantKirchhoffMaterial (real, self-consistent S/C)
    // since the production materials are still trace-mode placeholders
    // (S=0 always), which would make this check pass vacuously.
    for (fem::Hex8Element elem : {makeUnitCube(), makeDistortedHex()}) { // loop over both test geometries
        StVenantKirchhoffMaterial material(5.0, 2.0);
        const Eigen::VectorXd u = arbitrarySmallDisplacement();
        const Eigen::MatrixXd K = elem.computeTangentStiffness(u, material);
        const Eigen::MatrixXd Kfd = finiteDifferenceTangent(elem, u, material);
        EXPECT_TRUE(K.isApprox(Kfd, 1e-5)) << "max abs diff = " << (K - Kfd).cwiseAbs().maxCoeff();
    }
}

TEST(Hex8Element, PatchTestReproducesAnalyticalStressState) {
    // Homogeneous-deformation patch test: an affine field u(X) = (F0-I)*X
    // is exactly representable by trilinear shape functions, so F is
    // constant over the whole element and R must equal
    // integral(P0 * dN_a/dX0) dV0 with P0 = F0*S0 (1st Piola-Kirchhoff).
    // That reference value is built here directly from the public
    // shapeFunctions()/gaussPoints() API rather than by calling
    // computeResidual again, so this independently exercises the
    // B/Voigt bookkeeping inside computeResidual.
    fem::Hex8Element elem = makeUnitCube();
    const auto& X0 = unitCubeCorners();
    StVenantKirchhoffMaterial material(5.0, 2.0);

    Eigen::Matrix3d F0 = Eigen::Matrix3d::Identity();
    F0(0, 0) = 1.2;   // stretch along x
    F0(1, 1) = 0.95;  // slight compression along y
    F0(0, 1) = 0.05;  // a bit of shear

    Eigen::VectorXd u(24);
    for (int a = 0; a < 8; ++a) u.segment<3>(3 * a) = (F0 - Eigen::Matrix3d::Identity()) * X0[a]; // loop over nodes

    // Independent reference: integral(dN_a/dX0) dV0 per node, recomputed
    // here from shapeFunctions()/gaussPoints() rather than reusing
    // Hex8Element's private computeKinematics.
    std::array<Eigen::Vector3d, 8> intGradN;
    for (auto& g : intGradN) g.setZero();
    for (const auto& gp : elem.gaussPoints()) { // loop over Gauss points, accumulate the volume integral
        Eigen::VectorXd N;
        Eigen::MatrixXd dN_dxi;
        elem.shapeFunctions(gp.xi, N, dN_dxi);
        Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
        for (int a = 0; a < 8; ++a) J += X0[a] * dN_dxi.row(a); // loop over nodes, accumulate the reference Jacobian
        const double detJ = J.determinant();
        const Eigen::MatrixXd dN_dX0 = dN_dxi * J.inverse();
        for (int a = 0; a < 8; ++a) intGradN[a] += (gp.weight * detJ) * dN_dX0.row(a).transpose(); // loop over nodes
    }

    const Eigen::Matrix3d S0 = material.computeStress(F0);
    const Eigen::Matrix3d P0 = F0 * S0;
    const Eigen::VectorXd R = elem.computeResidual(u, material);
    for (int a = 0; a < 8; ++a) { // loop over nodes, compare each node's 3-vector force
        const Eigen::Vector3d expected = P0 * intGradN[a];
        EXPECT_TRUE(R.segment<3>(3 * a).isApprox(expected, 1e-8))
            << "node " << a << " expected " << expected.transpose()
            << " got " << R.segment<3>(3 * a).transpose();
    }
}

TEST(Hex8Element, DegenerateZeroVolumeElementProducesNonFiniteResidual) {
    // Flattening all "top" nodes onto the "bottom" face collapses the
    // element to zero volume, making the reference Jacobian singular.
    // Hex8Element does not guard against this (documented limitation: no
    // det(J) check before inverting in computeKinematics), so this test
    // locks in the current behavior — non-finite output — rather than
    // asserting a crash or a thrown exception.
    std::array<int, 8> ids = {0, 1, 2, 3, 4, 5, 6, 7};
    std::array<Eigen::Vector3d, 8> coords = {{
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, // top face collapsed onto the bottom face
    }};
    fem::Hex8Element elem(ids, coords);
    StVenantKirchhoffMaterial material(5.0, 2.0);
    const Eigen::VectorXd R = elem.computeResidual(Eigen::VectorXd::Zero(24), material);
    EXPECT_FALSE(R.allFinite());
}

TEST(Hex8Element, InvertedElementDoesNotThrow) {
    // Swapping the "bottom" (z=-1) and "top" (z=+1) physical corners while
    // keeping node indices fixed flips the element's orientation, giving a
    // uniformly negative (but non-singular) reference Jacobian determinant
    // ("inverted" element). Hex8Element does not detect or reject this
    // (documented limitation), so this only checks the computation runs to
    // completion and stays finite, not that any specific value is correct.
    std::array<int, 8> ids = {0, 1, 2, 3, 4, 5, 6, 7};
    std::array<Eigen::Vector3d, 8> coords = {{
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}, // physical "top" assigned to parametric "bottom" node indices
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, // physical "bottom" assigned to parametric "top" node indices
    }};
    fem::Hex8Element elem(ids, coords);
    StVenantKirchhoffMaterial material(5.0, 2.0);
    Eigen::VectorXd R;
    Eigen::MatrixXd K;
    EXPECT_NO_THROW(R = elem.computeResidual(arbitrarySmallDisplacement(), material));
    EXPECT_NO_THROW(K = elem.computeTangentStiffness(arbitrarySmallDisplacement(), material));
    EXPECT_TRUE(R.allFinite());
    EXPECT_TRUE(K.allFinite());
}

// ---------------------------------------------------------------------
// Tet4Element (still trace mode — see Tet4Element.cpp's file comment)
// ---------------------------------------------------------------------

TEST(Tet4Element, NodeIdsMatchConstructorArgument) {
    fem::Tet4Element elem = makeUnitTet();
    const std::vector<int> expected = {0, 1, 2, 3};
    EXPECT_EQ(elem.nodeIds(), expected);
}

TEST(Tet4Element, ShapeFunctionsSumToOneAndSatisfyKroneckerDeltaAtVertices) {
    fem::Tet4Element elem = makeUnitTet();
    Eigen::VectorXd N;
    Eigen::MatrixXd dN_dxi;
    for (auto xi : {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0),
                    Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.2, 0.3, 0.1)}) {
        elem.shapeFunctions(xi, N, dN_dxi);
        EXPECT_NEAR(N.sum(), 1.0, 1e-12);
    }
    const std::array<Eigen::Vector3d, 4> corners = {{
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
    }};
    for (int b = 0; b < 4; ++b) { // loop over the vertex being evaluated at
        elem.shapeFunctions(corners[b], N, dN_dxi);
        for (int a = 0; a < 4; ++a) EXPECT_NEAR(N(a), (a == b) ? 1.0 : 0.0, 1e-12); // loop over the 4 shape functions
    }
}

TEST(Tet4Element, ShapeFunctionGradientsAreConstant) {
    // Linear tet shape functions have xi-independent gradients; hardcoded
    // here to match the reference (barycentric) implementation.
    fem::Tet4Element elem = makeUnitTet();
    Eigen::VectorXd N;
    Eigen::MatrixXd dN_dxi;
    Eigen::MatrixXd expected(4, 3);
    expected << -1, -1, -1,
                 1,  0,  0,
                 0,  1,  0,
                 0,  0,  1;
    for (auto xi : {Eigen::Vector3d(0.25, 0.25, 0.25), Eigen::Vector3d(0.1, 0.6, 0.1)}) {
        elem.shapeFunctions(xi, N, dN_dxi);
        EXPECT_TRUE(dN_dxi.isApprox(expected, 1e-12));
    }
}

TEST(Tet4Element, HasSinglePointGaussRuleAtCentroidWithReferenceVolumeWeight) {
    fem::Tet4Element elem = makeUnitTet();
    ASSERT_EQ(elem.gaussPoints().size(), 1u);
    EXPECT_TRUE(elem.gaussPoints()[0].xi.isApprox(Eigen::Vector3d(0.25, 0.25, 0.25), 1e-12));
    EXPECT_NEAR(elem.gaussPoints()[0].weight, 1.0 / 6.0, 1e-12); // reference tet volume
}

TEST(Tet4Element, PlaceholderResidualHasCorrectSize) {
    fem::Tet4Element elem = makeUnitTet();
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    Eigen::VectorXd R = elem.computeResidual(Eigen::VectorXd::Zero(12), mat);
    EXPECT_EQ(R.size(), 12);
}

TEST(Tet4Element, PlaceholderTangentHasCorrectSize) {
    fem::Tet4Element elem = makeUnitTet();
    fem::NeoHookeanMaterial mat(1.0, 10.0);
    Eigen::MatrixXd K = elem.computeTangentStiffness(Eigen::VectorXd::Zero(12), mat);
    EXPECT_EQ(K.rows(), 12);
    EXPECT_EQ(K.cols(), 12);
}

TEST(Tet4Element, PatchTestReproducesAnalyticalStressState) {
    GTEST_SKIP() << "TODO: real computeResidual required. Not applicable while in trace mode.";
}

TEST(Tet4Element, TangentStiffnessMatchesFiniteDifferenceOfResidual) {
    GTEST_SKIP() << "MANDATORY before this element leaves trace mode: K_ij ~ "
                     "(R_i(u+eps*e_j)-R_i(u))/eps, as done above for Hex8Element.";
}
