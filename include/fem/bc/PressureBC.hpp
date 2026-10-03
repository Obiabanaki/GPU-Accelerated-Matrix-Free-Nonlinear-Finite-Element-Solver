/// @file PressureBC.hpp
/// @brief Follower pressure load on oriented quadrilateral surface facets.
#pragma once
#include <array>
#include <vector>
#include "fem/bc/BoundaryCondition.hpp"

namespace fem {

/// @brief One oriented Hex8 boundary facet in reference coordinates.
struct PressureFacet {
    std::array<int, 4> nodeIds;
    std::array<Eigen::Vector3d, 4> referenceCoordinates;
};

/// @brief Applies pressure traction and its consistent follower-load tangent.
class PressureBC final : public BoundaryCondition {
public:
    /// @param facets Oriented surface quads; normals point out of the body.
    /// @param pressure Positive pressure magnitude.
    PressureBC(std::vector<PressureFacet> facets, double pressure);

    /// @copydoc BoundaryCondition::apply
    void apply(GlobalSystem& system, const Eigen::VectorXd& displacement,
               double loadFactor) const override;

private:
    std::vector<PressureFacet> facets_;
    double pressure_;
};

} // namespace fem