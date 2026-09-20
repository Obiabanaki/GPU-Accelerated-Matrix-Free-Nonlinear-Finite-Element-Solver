/// @file Hex8Element.cpp
/// @brief Implementation of Hex8Element: total-Lagrangian trilinear hexahedron.
///
/// Node ordering convention (VTK_HEXAHEDRON layout, used by MeshBuilder):
///   0:(-1,-1,-1) 1:(1,-1,-1) 2:(1,1,-1) 3:(-1,1,-1)
///   4:(-1,-1, 1) 5:(1,-1, 1) 6:(1,1, 1) 7:(-1,1, 1)
///
/// computeResidual/computeTangentStiffness follow the standard total-
/// Lagrangian formulation: R = integral(B^T * S_voigt) dV0 and
/// K = integral(B^T * C * B) dV0 + geometric ("initial stress") stiffness,
/// with the Green-Lagrange strain-displacement matrix B built from the
/// current deformation gradient F. Voigt order is [11,22,33,12,23,13],
/// with strain rows 4-6 carrying the engineering (2x) shear factor so
/// that B^T * S_voigt reproduces the virtual work integral dE:S exactly.
#include "fem/element/Hex8Element.hpp"
#include <cmath>

namespace fem {

namespace { // internal linkage: file-local helpers, not part of Hex8Element's public API

constexpr std::array<std::array<double, 3>, 8> kLocalCorners = {{
    {-1, -1, -1}, { 1, -1, -1}, { 1,  1, -1}, {-1,  1, -1},
    {-1, -1,  1}, { 1, -1,  1}, { 1,  1,  1}, {-1,  1,  1},
}};

/// @brief Green-Lagrange strain-displacement matrix at one Gauss point.
/// @param dN_dX0 Shape function gradients w.r.t. reference coords (8x3), from computeKinematics.
/// @param F Deformation gradient at the same Gauss point, from computeKinematics.
/// @return B (6x24) such that dE_voigt = B * du; used by computeResidual (B^T*S)
/// and computeTangentStiffness (B^T*C*B). Assumes F and dN_dX0 come from the same
/// Gauss point/displacement state; not valid if mixed across points.
Eigen::Matrix<double, 6, 24> strainDisplacementMatrix(const Eigen::MatrixXd& dN_dX0,
                                                       const Eigen::Matrix3d& F) {
    Eigen::Matrix<double, 6, 24> B = Eigen::Matrix<double, 6, 24>::Zero();
    for (int a = 0; a < 8; ++a) { // loop over the element's 8 nodes
        const Eigen::RowVector3d g = dN_dX0.row(a);
        for (int j = 0; j < 3; ++j) { // loop over node a's 3 spatial displacement components
            const int col = 3 * a + j;
            B(0, col) = F(j, 0) * g(0);
            B(1, col) = F(j, 1) * g(1);
            B(2, col) = F(j, 2) * g(2);
            B(3, col) = F(j, 0) * g(1) + F(j, 1) * g(0);
            B(4, col) = F(j, 1) * g(2) + F(j, 2) * g(1);
            B(5, col) = F(j, 0) * g(2) + F(j, 2) * g(0);
        }
    }
    return B;
}

/// @brief Flatten a symmetric 3x3 tensor to Voigt order [11,22,33,12,23,13].
/// @param S Symmetric 3x3 tensor (only upper triangle is read); used here for the
/// 2nd Piola-Kirchhoff stress before contracting with strainDisplacementMatrix's B.
/// @return 6x1 Voigt vector, no engineering-shear doubling (stress, not strain).
Eigen::Matrix<double, 6, 1> toVoigt(const Eigen::Matrix3d& S) {
    Eigen::Matrix<double, 6, 1> v;
    v << S(0, 0), S(1, 1), S(2, 2), S(0, 1), S(1, 2), S(0, 2);
    return v;
}

} // namespace

Hex8Element::Hex8Element(std::array<int, 8> nodeIds, std::array<Eigen::Vector3d, 8> nodeCoords)
    : nodeIds_(nodeIds.begin(), nodeIds.end()), refCoords_(nodeCoords) { // members are initialized here before constructor runs
    // Standard 2x2x2 Gauss rule: 8 points at (+-1/sqrt(3))^3, weight 1 each.
    const double g = 1.0 / std::sqrt(3.0);
    // Triple range-based for, each over the 2-element list {-g,g}: enumerates all
    // 2x2x2=8 sign combinations (cartesian product) rather than 8 literal points.
    for (double gz : {-g, g}) {       // loop over the z-sign of each Gauss point
        for (double gy : {-g, g}) {   // loop over the y-sign
            for (double gx : {-g, g}) { // loop over the x-sign
                gaussPoints_.push_back(GaussPoint{Eigen::Vector3d(gx, gy, gz), 1.0});
            }
        }
    }
}

const std::vector<int>& Hex8Element::nodeIds() const { return nodeIds_; }

void Hex8Element::shapeFunctions(const Eigen::Vector3d& xi,
                                  Eigen::VectorXd& N,
                                  Eigen::MatrixXd& dN_dxi) const {
    N.resize(8);
    dN_dxi.resize(8, 3);
    for (int a = 0; a < 8; ++a) { // loop over the 8 trilinear shape functions/nodes
        const double xa = kLocalCorners[a][0];
        const double ya = kLocalCorners[a][1];
        const double za = kLocalCorners[a][2];
        const double fx = 1.0 + xa * xi(0);
        const double fy = 1.0 + ya * xi(1);
        const double fz = 1.0 + za * xi(2);

        N(a) = 0.125 * fx * fy * fz;
        dN_dxi(a, 0) = 0.125 * xa * fy * fz;
        dN_dxi(a, 1) = 0.125 * fx * ya * fz;
        dN_dxi(a, 2) = 0.125 * fx * fy * za;
    }
}

const std::vector<GaussPoint>& Hex8Element::gaussPoints() const { return gaussPoints_; }

// Shared by computeResidual and computeTangentStiffness so both use identical
// kinematics per Gauss point. Assumes u is ordered [ux0,uy0,uz0,ux1,...] matching
// nodeIds_/refCoords_, and that refCoords_ describes a non-degenerate hexahedron
// (J invertible); no check is done for inverted/degenerate elements.
void Hex8Element::computeKinematics(const Eigen::Vector3d& xi,
                                     const Eigen::VectorXd& u,
                                     Eigen::MatrixXd& dN_dX0,
                                     Eigen::Matrix3d& F,
                                     double& detJ) const {
    Eigen::VectorXd N;
    Eigen::MatrixXd dN_dxi;
    shapeFunctions(xi, N, dN_dxi);

    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    // loop over nodes, accumulate the reference Jacobian J = sum_a(X0_a outer dN/dxi_a)
    for (int a = 0; a < 8; ++a) J += refCoords_[a] * dN_dxi.row(a);
    detJ = J.determinant();
    dN_dX0 = dN_dxi * J.inverse();

    F = Eigen::Matrix3d::Identity();
    // loop over nodes, accumulate F = I + sum_a(u_a outer dN/dX0_a);
    // u.segment<3>(3*a) is Eigen's fixed-size sub-vector accessor, pulling node a's
    // 3 displacement dofs out of the flat 24-entry vector without copying the rest.
    for (int a = 0; a < 8; ++a) F += u.segment<3>(3 * a) * dN_dX0.row(a);
}

// Internal-force residual R(u) = integral(B^T * S_voigt) dV0, assembled per
// Gauss point via computeKinematics (F) -> material.computeStress (S) ->
// strainDisplacementMatrix (B). Feeds directly into NewtonSolver's residual;
// caller is responsible for combining this with external/body forces.
Eigen::VectorXd Hex8Element::computeResidual(const Eigen::VectorXd& u,
                                              const Material& material) const {
    Eigen::VectorXd R = Eigen::VectorXd::Zero(24);
    for (const GaussPoint& gp : gaussPoints_) { // loop over Gauss points, accumulate the volume integral
        Eigen::MatrixXd dN_dX0;
        Eigen::Matrix3d F;
        double detJ;
        computeKinematics(gp.xi, u, dN_dX0, F, detJ);

        const Eigen::Matrix3d S = material.computeStress(F);
        const Eigen::Matrix<double, 6, 24> B = strainDisplacementMatrix(dN_dX0, F);
        // .noalias(): R never appears on the RHS, so this tells Eigen to write
        // straight into R instead of allocating an aliasing-safety temporary.
        R.noalias() += (B.transpose() * toVoigt(S)) * (gp.weight * detJ);
    }
    return R;
}

// Consistent tangent K(u) = dR/du = material stiffness (B^T*C*B) + geometric
// (initial-stress) stiffness, matching computeResidual's R term-for-term so
// NewtonSolver's Jacobian is exact (see the mandatory finite-difference check
// in test_element.cpp). Assumes the same u ordering/kinematics as computeResidual.
Eigen::MatrixXd Hex8Element::computeTangentStiffness(const Eigen::VectorXd& u,
                                                      const Material& material) const {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(24, 24);
    for (const GaussPoint& gp : gaussPoints_) { // loop over Gauss points, accumulate the volume integral
        Eigen::MatrixXd dN_dX0;
        Eigen::Matrix3d F;
        double detJ;
        computeKinematics(gp.xi, u, dN_dX0, F, detJ);

        const Eigen::Matrix3d S = material.computeStress(F);
        const Eigen::Matrix<double, 6, 6> C = material.computeTangent(F);
        const Eigen::Matrix<double, 6, 24> B = strainDisplacementMatrix(dN_dX0, F);
        const double dV = gp.weight * detJ;

        K.noalias() += B.transpose() * C * B * dV; // see computeResidual for why .noalias() is safe here

        // Geometric (initial-stress) stiffness: scalar g_a.S.g_b times I_3 per node pair.
        // Loop over all 8x8 node pairs (a,b); K.block<3,3>(3a,3b) is Eigen's fixed-size
        // accessor into the 3x3 sub-block of K owned by that pair, avoiding a copy.
        for (int a = 0; a < 8; ++a) {     // loop over "row" nodes
            for (int b = 0; b < 8; ++b) { // loop over "column" nodes
                const double kab = dN_dX0.row(a) * S * dN_dX0.row(b).transpose();
                K.block<3, 3>(3 * a, 3 * b) += (kab * dV) * Eigen::Matrix3d::Identity();
            }
        }
    }
    return K;
}

} // namespace fem
