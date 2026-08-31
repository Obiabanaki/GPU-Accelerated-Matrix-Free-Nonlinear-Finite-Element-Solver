/// @file Hex8Element.hpp
/// @brief Concrete 8-node trilinear hexahedral Element implementation.
#pragma once
#include <array>
#include "fem/element/Element.hpp"

namespace fem {

/// @brief 8-node trilinear hexahedral element.
///
/// Step 0-pre / Phase 0.5: first concrete Element, used with the patch
/// test and the Phase 1 structured-mesh benchmark.
class Hex8Element : public Element {
public:
    /// @brief Construct from global node indices and reference coordinates.
    /// @param nodeIds Global indices of this element's 8 nodes.
    /// @param nodeCoords Reference-configuration coordinates of those 8 nodes.
    Hex8Element(std::array<int, 8> nodeIds, std::array<Eigen::Vector3d, 8> nodeCoords);

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
    std::vector<int> nodeIds_;                    ///< Global node indices (stored as vector to match Element's interface).
    std::array<Eigen::Vector3d, 8> refCoords_;     ///< Reference-configuration coordinates of the 8 nodes.
    std::vector<GaussPoint> gaussPoints_;          ///< Cached 2x2x2 Gauss rule, built once at construction.
};

} // namespace fem
