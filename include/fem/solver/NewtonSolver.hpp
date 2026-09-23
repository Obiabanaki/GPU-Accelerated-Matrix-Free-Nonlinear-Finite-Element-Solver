/// @file NewtonSolver.hpp
/// @brief Newton-Raphson orchestrator in the current facade pass.
///
/// The current implementation is intentionally a tracing facade: the outer
/// load-step / Newton-iteration loop is real, but the underlying constitutive
/// updates and linear solves remain placeholder implementations. This class is
/// therefore a genuine orchestration layer for sequencing assembly, boundary
/// conditions, and solve calls, while still exercising the object graph and
/// call order without computing a physically meaningful solution.
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
/// abstract interface. The current implementation logs the real call sequence
/// and advances the iterate, but the convergence criterion is still a
/// placeholder rather than the eventual residual/displacement-based logic.
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

    /// @brief Run the current facade-trace Newton solve loop.
    ///
    /// The method genuinely iterates over the requested load steps and Newton
    /// iterations, invoking the configured collaborators in the expected order.
    /// However, it still uses the placeholder convergence check and does not yet
    /// implement step-halving, residual norms, or a fully converged nonlinear
    /// solve.
    /// @param numLoadSteps Number of load-step iterations to trace.
    /// @param residualTol Placeholder residual tolerance retained for API parity.
    /// @param dispTol Placeholder displacement tolerance retained for API parity.
    /// @param maxIterPerStep Newton iteration cap per load step.
    /// @return Displacement field after the current trace pass; not yet a
    /// physically converged solution.
    Eigen::VectorXd solve(int numLoadSteps, double residualTol, double dispTol,
                           int maxIterPerStep);

    /// @brief Residual norm history for the current tracing pass.
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
