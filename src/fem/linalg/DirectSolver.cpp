/// @file DirectSolver.cpp
/// @brief Sparse LU implementation of DirectSolver.
#include "fem/linalg/DirectSolver.hpp"
#include "fem/linalg/EigenSparseOperator.hpp"
#include <Eigen/SparseLU>
#include <stdexcept>

namespace fem::linalg {

Eigen::VectorXd DirectSolver::solve(LinearOperator& op, const Eigen::VectorXd& R) {
    auto* sparseOp = dynamic_cast<EigenSparseOperator*>(&op);
    if (sparseOp == nullptr) {
        throw std::logic_error("DirectSolver requires an EigenSparseOperator");
    }
    if (R.size() != op.size()) {
        throw std::invalid_argument("DirectSolver::solve: right-hand side size does not match operator");
    }

    Eigen::SparseLU<Eigen::SparseMatrix<double>> factorization;
    factorization.compute(sparseOp->matrix());
    if (factorization.info() != Eigen::Success) {
        throw std::runtime_error("DirectSolver::solve: sparse factorization failed");
    }
    Eigen::VectorXd solution = factorization.solve(R);
    if (factorization.info() != Eigen::Success || !solution.allFinite()) {
        throw std::runtime_error("DirectSolver::solve: sparse solve failed");
    }
    lastStats_ = SolverStats{};
    return solution;
}

SolverStats DirectSolver::lastSolveStats() const { return lastStats_; }

} // namespace fem::linalg
