/// @file NewtonSolver.hpp
/// @brief Newton-Raphson orchestrator (NewtonSolver) — see ARCHITECTURE.md §9.
#pragma once
#include <functional>
#include <vector>
#include "fem/mesh/Mesh.hpp"
#include "fem/bc/BoundaryCondition.hpp"
#include "fem/linalg/LinearSolver.hpp"

namespace fem {

/// @brief Orchestrates the Newton-Raphson loop with load stepping.
///
/// Composed of (not inherited from) a Mesh, Material, LinearSolver, and
/// BoundaryConditions — every collaborator is touched only through its
/// abstract interface. Grep this class for concrete type names as a
/// design check: there should be none.
class NewtonSolver {
public:
    /// @brief Construct from references to every collaborator; NewtonSolver
    /// owns none of them (Rule of Zero — see ARCHITECTURE.md §9).
    /// @param mesh Mesh to assemble against.
    /// @param material Constitutive model passed through to Mesh::assemble.
    /// @param linearSolver Strategy used to solve each Newton increment.
    /// @param boundaryConditions Non-owning references to the BCs applied
    /// every iteration, in order.
    NewtonSolver(Mesh& mesh, Material& material, linalg::LinearSolver& linearSolver,
                 std::vector<std::reference_wrapper<BoundaryCondition>> boundaryConditions);

    /// @brief Run the full load-stepped Newton solve.
    /// @param numLoadSteps Number of increments to ramp load/prescribed displacement.
    /// @param residualTol Convergence tolerance on residual norm.
    /// @param dispTol Convergence tolerance on displacement increment norm.
    /// @param maxIterPerStep Newton iteration cap per load step; on
    /// non-convergence, halve the step and retry (Step 1.4 cutback logic)
    /// down to a configurable minimum before declaring failure.
    /// @return Final converged global displacement field.
    Eigen::VectorXd solve(int numLoadSteps, double residualTol, double dispTol,
                           int maxIterPerStep);

    /// @brief Residual norm per iteration, grouped by load step — source
    /// data for Step 1.4's quadratic-convergence plot.
    /// @return Reference to the recorded convergence history.
    const std::vector<std::vector<double>>& convergenceHistory() const {
        return convergenceHistory_;
    }

private:
    Mesh& mesh_;                                                          ///< Mesh assembled against every iteration; not owned.
    Material& material_;                                                  ///< Constitutive model used for assembly; not owned.
    linalg::LinearSolver& linearSolver_;                                  ///< Strategy for solving each Newton increment; not owned.
    std::vector<std::reference_wrapper<BoundaryCondition>> boundaryConditions_; ///< BCs applied every iteration, in order; not owned.
    std::vector<std::vector<double>> convergenceHistory_;                 ///< Residual norm per iteration, grouped by load step.
};

} // namespace fem
