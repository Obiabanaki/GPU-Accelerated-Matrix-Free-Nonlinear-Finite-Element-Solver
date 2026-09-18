/// @file Tet4Element.cpp
/// @brief Implementation of Tet4Element. TRACE MODE — see Hex8Element.cpp's
/// file comment for the rationale.
#include "fem/element/Tet4Element.hpp"
#include <iostream>

namespace fem {

Tet4Element::Tet4Element(std::array<int, 4> nodeIds, std::array<Eigen::Vector3d, 4> nodeCoords)
    : nodeIds_(nodeIds.begin(), nodeIds.end()), refCoords_(nodeCoords) {
    // Single-point rule at the centroid, weight = 1/6 (reference tet volume).
    // Real (quadrature setup, not physics).
    gaussPoints_.push_back(GaussPoint{Eigen::Vector3d(0.25, 0.25, 0.25), 1.0 / 6.0});
}

const std::vector<int>& Tet4Element::nodeIds() const { return nodeIds_; }

Eigen::VectorXd Tet4Element::computeResidual(const Eigen::VectorXd& u,
                                              const Material& material) const {
    std::cout << "[Tet4Element::computeResidual] would integrate over "
              << gaussPoints_.size() << " Gauss point(s) into a 12-vector (trace mode)\n";
    (void)u; (void)material;
    return Eigen::VectorXd::Zero(12);
}

Eigen::MatrixXd Tet4Element::computeTangentStiffness(const Eigen::VectorXd& u,
                                                      const Material& material) const {
    std::cout << "[Tet4Element::computeTangentStiffness] would assemble a 12x12 "
              << "tangent stiffness matrix (trace mode)\n";
    (void)u; (void)material;
    return Eigen::MatrixXd::Zero(12, 12);
}

void Tet4Element::shapeFunctions(const Eigen::Vector3d& xi,
                                  Eigen::VectorXd& N,
                                  Eigen::MatrixXd& dN_dxi) const {
    // Real linear barycentric shape functions.
    N.resize(4);
    dN_dxi.resize(4, 3);
    N(0) = 1.0 - xi(0) - xi(1) - xi(2);
    N(1) = xi(0);
    N(2) = xi(1);
    N(3) = xi(2);
    dN_dxi << -1, -1, -1,
               1,  0,  0,
               0,  1,  0,
               0,  0,  1;
}

const std::vector<GaussPoint>& Tet4Element::gaussPoints() const { return gaussPoints_; }

} // namespace fem
