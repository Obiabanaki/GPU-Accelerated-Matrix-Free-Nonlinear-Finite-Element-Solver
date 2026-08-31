/// @file Tet4Element.hpp
/// @brief Concrete 4-node linear tetrahedral Element implementation.
#pragma once
#include <array>
#include "fem/element/Element.hpp"

namespace fem {

/// @brief 4-node linear tetrahedral element.
///
/// Step 1.3: second Element subclass; must be mixable with Hex8Element in
/// the same Mesh, since Mesh only ever stores unique_ptr<Element>.
class Tet4Element : public Element {
public:
    /// @brief Construct from global node indices and reference coordinates.
    /// @param nodeIds Global indices of this element's 4 nodes.
    /// @param nodeCoords Reference-configuration coordinates of those 4 nodes.
    Tet4Element(std::array<int, 4> nodeIds, std::array<Eigen::Vector3d, 4> nodeCoords);

    /// @copydoc Element::nodeIds
    const std::vector<int>& nodeIds() const override;

    /// @copydoc Element::computeResidual
    Eigen::VectorXd computeResidual(const Eigen::VectorXd& u,
                                     const Material& material) const override;

    /// @copydoc Element::computeTangentStiffness
    Eigen::MatrixXd computeTangentStiffness(const Eigen::VectorXd& u,
                                             const Material& material) const override;

    /// @copydoc Element::shapeFunctions
    void shapeFunctions(const Eigen::Vector3d& xi,
                         Eigen::VectorXd& N,
                         Eigen::MatrixXd& dN_dxi) const override;

    /// @copydoc Element::gaussPoints
    const std::vector<GaussPoint>& gaussPoints() const override;

private:
    std::vector<int> nodeIds_;                     ///< Global node indices (stored as vector to match Element's interface).
    std::array<Eigen::Vector3d, 4> refCoords_;      ///< Reference-configuration coordinates of the 4 nodes.
    std::vector<GaussPoint> gaussPoints_;           ///< Cached quadrature rule (single-point rule is common for Tet4).
};

} // namespace fem
