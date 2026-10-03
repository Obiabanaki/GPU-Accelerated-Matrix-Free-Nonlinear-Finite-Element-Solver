/// @file NewtonSolver.hpp
/// @brief Load-stepped Newton-Raphson solver for nonlinear finite-element systems.
#pragma once
#include <functional>
#include <vector>
#include "fem/mesh/Mesh.hpp"
#include "fem/bc/BoundaryCondition.hpp"
#include "fem/linalg/LinearSolver.hpp"

namespace fem {

/// @brief Solves the assembled nonlinear equilibrium equations with load stepping.
///
/// Composed of (not inherited from) a Mesh, Material, LinearSolver, and
/// BoundaryConditions — every collaborator is touched only through its
/// abstract interface. Boundary values are ramped over the requested load
/// steps. A failed increment is halved and retried, with at most 12
/// consecutive cutbacks for each nominal step.
class NewtonSolver {
public:
    /// @brief Construct from references to every collaborator; NewtonSolver
    /// owns none of them.
    /// @param mesh Mesh to assemble against.
    /// @param material Constitutive model passed through to Mesh::assemble.
    /// @param linearSolver Strategy used to solve each Newton increment.
    /// @param boundaryConditions Non-owning references to the BCs applied
    /// every iteration, in order.
    NewtonSolver(Mesh& mesh, Material& material, linalg::LinearSolver& linearSolver,
                 std::vector<std::reference_wrapper<BoundaryCondition>> boundaryConditions);

    /// @brief Run a load-stepped Newton solve.
    /// @param numLoadSteps Number of nominal increments for prescribed displacements.
    /// @param residualTol Tolerance on the norm of the constrained Newton right-hand side.
    /// @param dispTol Tolerance on the displacement increment norm.
    /// @param maxIterPerStep Maximum Newton updates allowed for each load step.
    /// @return Converged global displacement field.
    /// @throws std::invalid_argument for invalid controls or tolerances.
    /// @throws std::runtime_error if a load step does not converge or a solve fails.
    Eigen::VectorXd solve(int numLoadSteps, double residualTol, double dispTol,
                           int maxIterPerStep);

    /// @brief Constrained Newton right-hand-side norm per iteration, grouped by load step.
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
