/// @file JacobiPreconditioner.cpp
/// @brief Implementation of JacobiPreconditioner. TRACE MODE.
#include "fem/linalg/JacobiPreconditioner.hpp"
#include <iostream>

namespace fem::linalg {

void JacobiPreconditioner::setup(const LinearOperator& op) {
    std::cout << "[JacobiPreconditioner::setup] would cache the inverse diagonal "
              << "of a " << op.size() << "x" << op.size() << " operator (trace mode)\n";
    invDiag_ = Eigen::VectorXd::Zero(op.size());
}

Eigen::VectorXd JacobiPreconditioner::apply(const Eigen::VectorXd& r) const {
    std::cout << "[JacobiPreconditioner::apply] would scale by cached inverse diagonal (trace mode)\n";
    return r;
}

} // namespace fem::linalg
