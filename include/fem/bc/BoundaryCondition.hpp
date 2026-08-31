/// @file BoundaryCondition.hpp
/// @brief Abstract post-assembly constraint interface (BoundaryCondition).
#pragma once
#include "fem/mesh/GlobalSystem.hpp"

namespace fem {

/// @brief Modifies the global system to enforce a constraint after assembly.
class BoundaryCondition {
public:
    /// @brief Virtual destructor; BoundaryCondition is always used polymorphically.
    virtual ~BoundaryCondition() = default;

    /// @brief Apply this BC in place to the assembled global system.
    /// @param system Global system to modify (already assembled, pre-finalize
    /// or post-finalize depending on the concrete BC's requirements).
    virtual void apply(GlobalSystem& system) const = 0;
};

} // namespace fem
