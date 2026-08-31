/// @file DirichletBC.hpp
/// @brief Concrete prescribed-displacement BoundaryCondition.
#pragma once
#include <vector>
#include "fem/bc/BoundaryCondition.hpp"

namespace fem {

/// @brief Prescribes displacement at specific DOFs via row/column elimination.
///
/// Step 1.4. Must be applied with the INCREMENTAL prescribed value at each
/// Newton iteration within a load step, not the total, or load stepping
/// silently applies the full displacement in one shot — see ARCHITECTURE.md.
class DirichletBC : public BoundaryCondition {
public:
    /// @brief Construct from a list of DOFs and their prescribed values.
    /// @param dofs Global DOF indices to constrain.
    /// @param prescribedValues Target displacement value for each DOF in
    /// dofs, same order and length.
    DirichletBC(std::vector<int> dofs, std::vector<double> prescribedValues);

    /// @copydoc BoundaryCondition::apply
    void apply(GlobalSystem& system) const override;

private:
    std::vector<int> dofs_;        ///< Global DOF indices constrained by this BC.
    std::vector<double> values_;   ///< Prescribed displacement value per DOF, matching dofs_.
};

} // namespace fem
