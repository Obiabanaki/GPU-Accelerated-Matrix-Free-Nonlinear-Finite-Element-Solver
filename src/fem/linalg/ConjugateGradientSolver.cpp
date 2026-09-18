/// @file ConjugateGradientSolver.cpp
/// @brief Implementation of ConjugateGradientSolver. TRACE MODE.
#include "fem/linalg/ConjugateGradientSolver.hpp"
#include <iostream>

namespace fem::linalg {

ConjugateGradientSolver::ConjugateGradientSolver(Preconditioner& preconditioner,
                                                  double tol, int maxIter)
    : preconditioner_(preconditioner), tol_(tol), maxIter_(maxIter) {}

Eigen::VectorXd ConjugateGradientSolver::solve(LinearOperator& op, const Eigen::VectorXd& R) {
    std::cout << "[ConjugateGradientSolver::solve] would run matrix-free CG (tol=" << tol_
              << ", maxIter=" << maxIter_ << ") on a " << op.size()
              << "x" << op.size() << " system via op.applyTo(x) only (trace mode)\n";
    preconditioner_.setup(op);
    lastStats_ = SolverStats{};
    return Eigen::VectorXd::Zero(R.size());
}

SolverStats ConjugateGradientSolver::lastSolveStats() const { return lastStats_; }

} // namespace fem::linalg
