/// @file ConjugateGradientSolver.hpp
/// @brief Concrete matrix-free Conjugate Gradient LinearSolver implementation.
#pragma once
#include "fem/linalg/LinearSolver.hpp"
#include "fem/linalg/Preconditioner.hpp"

namespace fem::linalg {

/// @brief Matrix-free, backend-agnostic Conjugate Gradient.
///
/// Step 2.1. Deliberately does NOT take a ComputeBackend& — that would
/// break the Strategy separation. CG only ever calls op.applyTo(x); the
/// backend, if any, is entirely hidden inside whichever LinearOperator
/// it's handed at solve() time (EigenSparseOperator or BackendOperator).
class ConjugateGradientSolver : public LinearSolver {
public:
    /// @brief Construct with an injected preconditioner and convergence controls.
    /// @param preconditioner Injected at construction (fixed strategy for
    /// the solver's lifetime) — contrast with LinearOperator, passed to
    /// solve() because a fresh one is built every Newton iteration.
    /// @param tol Relative residual-norm tolerance for convergence.
    /// @param maxIter Maximum CG iterations before giving up.
    explicit ConjugateGradientSolver(Preconditioner& preconditioner,
                                      double tol = 1e-8, int maxIter = 1000);

    /// @copydoc LinearSolver::solve
    Eigen::VectorXd solve(LinearOperator& op, const Eigen::VectorXd& R) override;

    /// @copydoc LinearSolver::lastSolveStats
    SolverStats lastSolveStats() const override;

private:
    Preconditioner& preconditioner_;   ///< Injected preconditioner strategy; not owned.
    double tol_;                       ///< Relative residual-norm convergence tolerance.
    int maxIter_;                      ///< Maximum CG iterations.
    SolverStats lastStats_;            ///< Stats (iteration count, residual history) from the most recent solve().
};

} // namespace fem::linalg
