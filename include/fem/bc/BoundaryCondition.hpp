/// @file BoundaryCondition.hpp
/// @brief Abstract post-assembly constraint interface (BoundaryCondition).
#pragma once
#include <Eigen/Dense>
#include "fem/mesh/GlobalSystem.hpp"

namespace fem {

/// @brief Modifies the global system to enforce a constraint after assembly.
class BoundaryCondition {
public:
    /// @brief Virtual destructor; BoundaryCondition is always used polymorphically.
    virtual ~BoundaryCondition() = default;

    /// @brief Apply this BC to the assembled Newton system.
    /// @param system Assembled system whose residual contains the Newton right-hand side (-R).
    /// @param displacement Current global displacement iterate.
    /// @param loadFactor Fraction of the final prescribed displacement for this load step.
    virtual void apply(GlobalSystem& system, const Eigen::VectorXd& displacement,
                       double loadFactor) const = 0;
};

} // namespace fem
