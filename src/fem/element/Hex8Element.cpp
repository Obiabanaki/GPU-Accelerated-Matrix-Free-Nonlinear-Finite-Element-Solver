/// @file Hex8Element.cpp
/// @brief Implementation of Hex8Element.
///
/// Node ordering convention (VTK_HEXAHEDRON layout, used by MeshBuilder):
///   0:(-1,-1,-1) 1:(1,-1,-1) 2:(1,1,-1) 3:(-1,1,-1)
///   4:(-1,-1, 1) 5:(1,-1, 1) 6:(1,1, 1) 7:(-1,1, 1)
///
/// TRACE MODE: the constructor and shapeFunctions() are real (pure
/// geometry/topology, not physics), but computeResidual and
/// computeTangentStiffness only print what they would do and return
/// placeholder zero values. A full, FD-verified total-Lagrangian
/// implementation (real Jacobian, real F, real material+geometric
/// stiffness) was written in an earlier pass — ask to have it merged back
/// in once the pipeline wiring is validated.
#include "fem/element/Hex8Element.hpp"
#include <cmath>
#include <iostream>

namespace fem {

namespace {

constexpr std::array<std::array<double, 3>, 8> kLocalCorners = {{
    {-1, -1, -1}, { 1, -1, -1}, { 1,  1, -1}, {-1,  1, -1},
    {-1, -1,  1}, { 1, -1,  1}, { 1,  1,  1}, {-1,  1,  1},
}};

} // namespace

Hex8Element::Hex8Element(std::array<int, 8> nodeIds, std::array<Eigen::Vector3d, 8> nodeCoords)
    : nodeIds_(nodeIds.begin(), nodeIds.end()), refCoords_(nodeCoords) {
    // Standard 2x2x2 Gauss rule: 8 points at (+-1/sqrt(3))^3, weight 1 each.
    // Real (topology/quadrature setup, not physics), so it's kept as-is.
    const double g = 1.0 / std::sqrt(3.0);
    for (double gz : {-g, g}) {
        for (double gy : {-g, g}) {
            for (double gx : {-g, g}) {
                gaussPoints_.push_back(GaussPoint{Eigen::Vector3d(gx, gy, gz), 1.0});
            }
        }
    }
}

const std::vector<int>& Hex8Element::nodeIds() const { return nodeIds_; }

void Hex8Element::shapeFunctions(const Eigen::Vector3d& xi,
                                  Eigen::VectorXd& N,
                                  Eigen::MatrixXd& dN_dxi) const {
    // Real trilinear shape functions — pure reference-element geometry,
    // needed even in trace mode for anything that wants element shape.
    N.resize(8);
    dN_dxi.resize(8, 3);
    for (int a = 0; a < 8; ++a) {
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

Eigen::VectorXd Hex8Element::computeResidual(const Eigen::VectorXd& u,
                                              const Material& material) const {
    std::cout << "[Hex8Element::computeResidual] would loop over " << gaussPoints_.size()
              << " Gauss points, compute F, call material.computeStress(F), and integrate "
              << "P*dN_dX0 into a 24-vector (trace mode)\n";
    (void)u; (void)material;
    return Eigen::VectorXd::Zero(24);
}

Eigen::MatrixXd Hex8Element::computeTangentStiffness(const Eigen::VectorXd& u,
                                                      const Material& material) const {
    std::cout << "[Hex8Element::computeTangentStiffness] would assemble material + "
              << "geometric stiffness into a 24x24 matrix (trace mode)\n";
    (void)u; (void)material;
    return Eigen::MatrixXd::Zero(24, 24);
}

} // namespace fem
