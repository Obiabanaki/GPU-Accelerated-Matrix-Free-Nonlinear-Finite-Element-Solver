/// @file DirectSolver.cpp
/// @brief Implementation of DirectSolver.
#include "fem/linalg/DirectSolver.hpp"
#include "fem/linalg/EigenSparseOperator.hpp"
#include <Eigen/SparseLU>
#include <stdexcept>

namespace fem::linalg {

Eigen::VectorXd DirectSolver::solve(LinearOperator& op, const Eigen::VectorXd& R) {
    // TODO(Phase 1): dynamic_cast<EigenSparseOperator*>(&op) to reach the
    // underlying Eigen::SparseMatrix, factorize with Eigen::SparseLU, solve.
    // Throw a clear error if op isn't an EigenSparseOperator — a direct
    // factorization of a matrix-free/backend operator makes no sense.
    (void)op;
    lastStats_ = SolverStats{};
    return Eigen::VectorXd::Zero(R.size());
}

SolverStats DirectSolver::lastSolveStats() const { return lastStats_; }

} // namespace fem::linalg
