/// @file JacobiPreconditioner.cpp
/// @brief Implementation of JacobiPreconditioner.
#include "fem/linalg/JacobiPreconditioner.hpp"

namespace fem::linalg {

void JacobiPreconditioner::setup(const LinearOperator& op) {
    // TODO(Step 2.2): invDiag_ = op.diagonal().cwiseInverse(); assert this
    // is called once per Newton iteration, not once per CG iteration.
    Eigen::VectorXd d = op.diagonal();
    invDiag_ = d.unaryExpr([](double v) { return v != 0.0 ? 1.0 / v : 0.0; });
}

Eigen::VectorXd JacobiPreconditioner::apply(const Eigen::VectorXd& r) const {
    return invDiag_.cwiseProduct(r);
}

} // namespace fem::linalg
