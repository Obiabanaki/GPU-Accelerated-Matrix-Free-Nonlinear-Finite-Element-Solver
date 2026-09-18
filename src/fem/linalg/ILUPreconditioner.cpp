/// @file ILUPreconditioner.cpp
/// @brief Implementation of ILUPreconditioner. TRACE MODE.
#include "fem/linalg/ILUPreconditioner.hpp"
#include <iostream>

namespace fem::linalg {

void ILUPreconditioner::setup(const LinearOperator& op) {
    std::cout << "[ILUPreconditioner::setup] would factor a " << op.size()
              << "x" << op.size() << " matrix via Eigen::IncompleteLUT — CPU-only, "
              << "see ARCHITECTURE.md's matrix-free/preconditioner tension (trace mode)\n";
}

Eigen::VectorXd ILUPreconditioner::apply(const Eigen::VectorXd& r) const {
    std::cout << "[ILUPreconditioner::apply] would back-substitute through the ILU factors (trace mode)\n";
    return r;
}

} // namespace fem::linalg
