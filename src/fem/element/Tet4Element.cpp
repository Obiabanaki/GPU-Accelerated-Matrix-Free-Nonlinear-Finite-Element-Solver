/// @file Tet4Element.cpp
/// @brief Implementation of Tet4Element.
#include "fem/element/Tet4Element.hpp"

namespace fem {

Tet4Element::Tet4Element(std::array<int, 4> nodeIds, std::array<Eigen::Vector3d, 4> nodeCoords)
    : nodeIds_(nodeIds.begin(), nodeIds.end()), refCoords_(nodeCoords) {
    // TODO(Step 1.3): populate gaussPoints_ (single-point rule at the
    // centroid is common for linear Tet4, weight = element volume).
}

const std::vector<int>& Tet4Element::nodeIds() const { return nodeIds_; }

Eigen::VectorXd Tet4Element::computeResidual(const Eigen::VectorXd& u,
                                              const Material& material) const {
    (void)u; (void)material;
    return Eigen::VectorXd::Zero(12); // 4 nodes * 3 dof
}

Eigen::MatrixXd Tet4Element::computeTangentStiffness(const Eigen::VectorXd& u,
                                                      const Material& material) const {
    (void)u; (void)material;
    return Eigen::MatrixXd::Zero(12, 12);
}

void Tet4Element::shapeFunctions(const Eigen::Vector3d& xi,
                                  Eigen::VectorXd& N,
                                  Eigen::MatrixXd& dN_dxi) const {
    // TODO(Step 1.3): linear barycentric shape functions.
    (void)xi;
    N = Eigen::VectorXd::Zero(4);
    dN_dxi = Eigen::MatrixXd::Zero(4, 3);
}

const std::vector<GaussPoint>& Tet4Element::gaussPoints() const { return gaussPoints_; }

} // namespace fem
