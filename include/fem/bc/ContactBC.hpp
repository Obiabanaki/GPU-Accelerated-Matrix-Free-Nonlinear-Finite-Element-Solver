/// @file ContactBC.hpp
/// @brief Concrete penalty-based rigid-plane contact BoundaryCondition.
#pragma once
#include <vector>
#include <Eigen/Dense>
#include "fem/bc/BoundaryCondition.hpp"

namespace fem {

/// @brief Penalty-based contact against a rigid plane.
///
/// Step 2.3-stretch: explicitly optional. Adds an asymmetric stiffness/
/// force contribution when penetration is detected, which is why it's the
/// motivating case for GMRESSolver rather than ConjugateGradientSolver.
class ContactBC : public BoundaryCondition {
public:
    /// @brief Construct a penalty contact constraint against a rigid plane.
    /// @param candidateNodeDofs Global DOF indices of nodes that may contact the plane.
    /// @param planeNormal Unit normal vector of the rigid contact plane.
    /// @param planeOffset Signed distance of the plane from the origin along planeNormal.
    /// @param penaltyStiffness Penalty stiffness applied per unit penetration depth.
    ContactBC(std::vector<int> candidateNodeDofs, Eigen::Vector3d planeNormal,
              double planeOffset, double penaltyStiffness);

    /// @copydoc BoundaryCondition::apply
    void apply(GlobalSystem& system) const override;

private:
    std::vector<int> candidateDofs_;   ///< Global DOF indices of candidate contact nodes.
    Eigen::Vector3d planeNormal_;      ///< Unit normal of the rigid contact plane.
    double planeOffset_;               ///< Signed plane offset from the origin along planeNormal_.
    double penalty_;                   ///< Penalty stiffness per unit penetration depth.
};

} // namespace fem
