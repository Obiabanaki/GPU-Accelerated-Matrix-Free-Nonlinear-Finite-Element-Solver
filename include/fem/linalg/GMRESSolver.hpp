/// @file GMRESSolver.hpp
/// @brief Concrete matrix-free GMRES LinearSolver implementation.
#pragma once
#include "fem/linalg/LinearSolver.hpp"
#include "fem/linalg/Preconditioner.hpp"

namespace fem::linalg {

/// @brief Matrix-free GMRES for non-symmetric systems.
///
/// Step 2.3: motivated by a follower-load boundary condition that breaks
/// tangent-matrix symmetry (not by contact — contact is 2.3-stretch).
/// Same shape as ConjugateGradientSolver: op.applyTo(x) only.
class GMRESSolver : public LinearSolver {
public:
    /// @brief Construct with an injected preconditioner and convergence controls.
    /// @param preconditioner Injected at construction, same rationale as
    /// ConjugateGradientSolver's.
    /// @param tol Relative residual-norm tolerance for convergence.
    /// @param maxIter Maximum total GMRES iterations before giving up.
    /// @param restart Number of iterations between GMRES restarts.
    explicit GMRESSolver(Preconditioner& preconditioner,
                          double tol = 1e-8, int maxIter = 1000, int restart = 30);

    /// @copydoc LinearSolver::solve
    Eigen::VectorXd solve(LinearOperator& op, const Eigen::VectorXd& R) override;

    /// @copydoc LinearSolver::lastSolveStats
    SolverStats lastSolveStats() const override;

private:
    Preconditioner& preconditioner_;   ///< Injected preconditioner strategy; not owned.
    double tol_;                       ///< Relative residual-norm convergence tolerance.
    int maxIter_;                      ///< Maximum total GMRES iterations.
    int restart_;                      ///< Iterations between restarts.
    SolverStats lastStats_;            ///< Stats (iteration count, residual history) from the most recent solve().
};

} // namespace fem::linalg
