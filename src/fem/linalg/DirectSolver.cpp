/// @file DirectSolver.cpp
/// @brief Implementation of DirectSolver. TRACE MODE.
#include "fem/linalg/DirectSolver.hpp"
#include <iostream>

namespace fem::linalg {

Eigen::VectorXd DirectSolver::solve(LinearOperator& op, const Eigen::VectorXd& R) {
    std::cout << "[DirectSolver::solve] would factorize a " << op.size()
              << "x" << op.size() << " system via Eigen::SparseLU (trace mode)\n";
    lastStats_ = SolverStats{};
    return Eigen::VectorXd::Zero(R.size());
}

SolverStats DirectSolver::lastSolveStats() const { return lastStats_; }

} // namespace fem::linalg
