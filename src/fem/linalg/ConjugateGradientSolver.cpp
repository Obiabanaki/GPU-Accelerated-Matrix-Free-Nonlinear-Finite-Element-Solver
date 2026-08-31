/// @file ConjugateGradientSolver.cpp
/// @brief Implementation of ConjugateGradientSolver.
#include "fem/linalg/ConjugateGradientSolver.hpp"

namespace fem::linalg {

ConjugateGradientSolver::ConjugateGradientSolver(Preconditioner& preconditioner,
                                                  double tol, int maxIter)
    : preconditioner_(preconditioner), tol_(tol), maxIter_(maxIter) {}

Eigen::VectorXd ConjugateGradientSolver::solve(LinearOperator& op, const Eigen::VectorXd& R) {
    // TODO(Step 2.1): standard preconditioned CG, using ONLY op.applyTo(x)
    // and preconditioner_.apply(r) — never a concrete matrix type, never
    // backend::ComputeBackend directly. Call preconditioner_.setup(op)
    // once at the start, not once per CG iteration.
    //
    // preconditioner_.setup(op);
    // Eigen::VectorXd x = Eigen::VectorXd::Zero(op.size());
    // Eigen::VectorXd r = R - op.applyTo(x);
    // Eigen::VectorXd z = preconditioner_.apply(r);
    // Eigen::VectorXd p = z;
    // ... iterate until ||r|| < tol_ or maxIter_ reached, recording
    // lastStats_.residualHistory each iteration.
    preconditioner_.setup(op);
    lastStats_ = SolverStats{};
    return Eigen::VectorXd::Zero(R.size());
}

SolverStats ConjugateGradientSolver::lastSolveStats() const { return lastStats_; }

} // namespace fem::linalg
