/// @file DirichletBC.hpp
/// @brief Concrete prescribed-displacement BoundaryCondition.
#pragma once
#include <vector>
#include "fem/bc/BoundaryCondition.hpp"

namespace fem {

/// @brief Prescribes displacement at specific DOFs via row/column elimination.
///
/// At each Newton iteration, the remaining displacement to the current load
/// target is applied to the right-hand side while the constrained matrix rows
/// and columns are eliminated symmetrically.
class DirichletBC : public BoundaryCondition {
public:
    /// @brief Construct from a list of DOFs and their prescribed values.
    /// @param dofs Global DOF indices to constrain.
    /// @param prescribedValues Target displacement value for each DOF in
    /// dofs, same order and length.
    DirichletBC(std::vector<int> dofs, std::vector<double> prescribedValues);

    /// @copydoc BoundaryCondition::apply
    void apply(GlobalSystem& system, const Eigen::VectorXd& displacement,
               double loadFactor) const override;

private:
    std::vector<int> dofs_;        ///< Global DOF indices constrained by this BC.
    std::vector<double> values_;   ///< Prescribed displacement value per DOF, matching dofs_.
};

} // namespace fem
