/// @file GMRESSolver.cpp
/// @brief Implementation of GMRESSolver.
#include "fem/linalg/GMRESSolver.hpp"

namespace fem::linalg {

GMRESSolver::GMRESSolver(Preconditioner& preconditioner, double tol, int maxIter, int restart)
    : preconditioner_(preconditioner), tol_(tol), maxIter_(maxIter), restart_(restart) {}

Eigen::VectorXd GMRESSolver::solve(LinearOperator& op, const Eigen::VectorXd& R) {
    // TODO(Step 2.3): restarted GMRES via op.applyTo(x) only, same
    // matrix-free contract as ConjugateGradientSolver. Motivated by the
    // follower-load BC's non-symmetric tangent, not by contact.
    preconditioner_.setup(op);
    lastStats_ = SolverStats{};
    return Eigen::VectorXd::Zero(R.size());
}

SolverStats GMRESSolver::lastSolveStats() const { return lastStats_; }

} // namespace fem::linalg
