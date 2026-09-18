/// @file GMRESSolver.cpp
/// @brief Implementation of GMRESSolver. TRACE MODE.
#include "fem/linalg/GMRESSolver.hpp"
#include <iostream>

namespace fem::linalg {

GMRESSolver::GMRESSolver(Preconditioner& preconditioner, double tol, int maxIter, int restart)
    : preconditioner_(preconditioner), tol_(tol), maxIter_(maxIter), restart_(restart) {}

Eigen::VectorXd GMRESSolver::solve(LinearOperator& op, const Eigen::VectorXd& R) {
    std::cout << "[GMRESSolver::solve] would run restarted GMRES (tol=" << tol_
              << ", restart=" << restart_ << ") on a " << op.size()
              << "x" << op.size() << " system via op.applyTo(x) only (trace mode)\n";
    preconditioner_.setup(op);
    lastStats_ = SolverStats{};
    return Eigen::VectorXd::Zero(R.size());
}

SolverStats GMRESSolver::lastSolveStats() const { return lastStats_; }

} // namespace fem::linalg
