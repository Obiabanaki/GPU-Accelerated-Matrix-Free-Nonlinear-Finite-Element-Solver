/// @file Element.hpp
/// @brief Abstract finite-element interface (Element) and GaussPoint helper struct.
#pragma once
#include <vector>
#include <Eigen/Dense>
#include "fem/material/Material.hpp"

namespace fem {

/// @brief One Gauss quadrature point: parametric location and weight.
struct GaussPoint {
    Eigen::Vector3d xi;    ///< Location in parametric (reference-element) space.
    double weight;         ///< Quadrature weight.
};

/// @brief Abstract interface for one finite element's local kinematics
/// and integration.
///
/// An Element converts nodal displacements + a Material into a local
/// residual and tangent stiffness. It never owns a Material — one is
/// passed in per call — so a single Hex8Element type works unmodified
/// with any Material subclass.
class Element {
public:
    /// @brief Virtual destructor; Element is always used polymorphically.
    virtual ~Element() = default;

    /// @brief Global node indices this element connects (defines DOF mapping).
    /// @return Reference to the element's ordered list of global node indices.
    virtual const std::vector<int>& nodeIds() const = 0;

    /// @brief Local internal-force residual vector for nodal displacement u.
    /// @param u Nodal displacements for this element's nodes, local ordering.
    /// @param material Constitutive model evaluated at each Gauss point.
    /// @return Local residual vector, size = numNodes * numDofPerNode.
    virtual Eigen::VectorXd computeResidual(const Eigen::VectorXd& u,
                                             const Material& material) const = 0;

    /// @brief Local tangent stiffness (material stiffness + geometric
    /// stiffness — both are required; see ARCHITECTURE.md design check).
    /// @param u Nodal displacements for this element's nodes, local ordering.
    /// @param material Constitutive model evaluated at each Gauss point.
    /// @return Local tangent stiffness matrix, size = local DOF count squared.
    virtual Eigen::MatrixXd computeTangentStiffness(const Eigen::VectorXd& u,
                                                     const Material& material) const = 0;

    /// @brief Shape function values and parametric-space gradients at a point.
    /// @param xi Point in parametric (reference-element) space.
    /// @param[out] N Shape function values at xi, size = numNodes.
    /// @param[out] dN_dxi Shape function gradients w.r.t. parametric coords,
    /// size = numNodes x 3.
    virtual void shapeFunctions(const Eigen::Vector3d& xi,
                                 Eigen::VectorXd& N,
                                 Eigen::MatrixXd& dN_dxi) const = 0;

    /// @brief Gauss quadrature points/weights for this element's integration rule.
    /// @return Reference to the cached quadrature rule.
    virtual const std::vector<GaussPoint>& gaussPoints() const = 0;
};

} // namespace fem
