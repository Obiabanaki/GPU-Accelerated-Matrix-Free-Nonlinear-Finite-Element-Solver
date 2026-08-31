/// @file Hex8Element.cpp
/// @brief Implementation of Hex8Element.
#include "fem/element/Hex8Element.hpp"

namespace fem {

Hex8Element::Hex8Element(std::array<int, 8> nodeIds, std::array<Eigen::Vector3d, 8> nodeCoords)
    : nodeIds_(nodeIds.begin(), nodeIds.end()), refCoords_(nodeCoords) {
    // TODO(Phase 0-pre): populate gaussPoints_ with the standard 2x2x2
    // Gauss rule (8 points, weight 1.0 each in [-1,1]^3 parametric space).
}

const std::vector<int>& Hex8Element::nodeIds() const { return nodeIds_; }

Eigen::VectorXd Hex8Element::computeResidual(const Eigen::VectorXd& u,
                                              const Material& material) const {
    // TODO(Phase 0-pre): loop over gaussPoints_, compute F at each point via
    // shapeFunctions()/dN_dxi and nodal u, call material.computeStress(F),
    // contract against the B-matrix, accumulate weighted by |J|*weight.
    (void)u; (void)material;
    return Eigen::VectorXd::Zero(24); // 8 nodes * 3 dof
}

Eigen::MatrixXd Hex8Element::computeTangentStiffness(const Eigen::VectorXd& u,
                                                      const Material& material) const {
    // TODO(Phase 0-pre): material stiffness from material.computeTangent(F)
    // contracted through B^T*C*B, PLUS geometric stiffness from the current
    // stress state (sigma : grad(delta_u) . grad(u) term). Missing the
    // geometric term is the most common bug here — see ARCHITECTURE.md.
    // MANDATORY: verify with a finite-difference perturbation of
    // computeResidual before trusting this in Step 1.4's Newton loop.
    (void)u; (void)material;
    return Eigen::MatrixXd::Zero(24, 24);
}

void Hex8Element::shapeFunctions(const Eigen::Vector3d& xi,
                                  Eigen::VectorXd& N,
                                  Eigen::MatrixXd& dN_dxi) const {
    // TODO(Phase 0-pre): standard trilinear shape functions
    // N_i = 1/8 * (1 + xi_i*xi)(1 + eta_i*eta)(1 + zeta_i*zeta), i=1..8.
    (void)xi;
    N = Eigen::VectorXd::Zero(8);
    dN_dxi = Eigen::MatrixXd::Zero(8, 3);
}

const std::vector<GaussPoint>& Hex8Element::gaussPoints() const { return gaussPoints_; }

} // namespace fem
