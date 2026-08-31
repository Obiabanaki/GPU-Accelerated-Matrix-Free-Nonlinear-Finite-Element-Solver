/// @file DirectSolver.hpp
/// @brief Concrete direct (LU factorization) LinearSolver implementation.
#pragma once
#include "fem/linalg/LinearSolver.hpp"

namespace fem::linalg {

/// @brief Direct solve via dense/sparse LU. Phase 1's baseline solver.
///
/// Only ever used with an EigenSparseOperator in practice — there's no
/// such thing as a matrix-free direct factorization — but its signature
/// is identical to every other LinearSolver.
class DirectSolver : public LinearSolver {
public:
    /// @copydoc LinearSolver::solve
    /// @throws std::logic_error if op is not backed by an EigenSparseOperator.
    Eigen::VectorXd solve(LinearOperator& op, const Eigen::VectorXd& R) override;

    /// @copydoc LinearSolver::lastSolveStats
    SolverStats lastSolveStats() const override;

private:
    SolverStats lastStats_;    ///< Stats from the most recent solve() call (trivial: direct solves don't iterate).
};

} // namespace fem::linalg
