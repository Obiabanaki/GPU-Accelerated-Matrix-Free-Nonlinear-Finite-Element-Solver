/// @file ILUPreconditioner.cpp
/// @brief Implementation of ILUPreconditioner.
#include "fem/linalg/ILUPreconditioner.hpp"
#include <stdexcept>

namespace fem::linalg {

void ILUPreconditioner::setup(const LinearOperator& op) {
    // TODO(Step 2.2): ILUPreconditioner needs the full sparse matrix, which
    // pure LinearOperator::applyTo can't provide. In practice this means
    // ILUPreconditioner is only ever setup() with an EigenSparseOperator;
    // consider a dynamic_cast + informative throw for BackendOperator,
    // documented as the CPU-only scope decision from ARCHITECTURE.md §7.
    (void)op;
    throw std::logic_error("ILUPreconditioner::setup not yet implemented");
}

Eigen::VectorXd ILUPreconditioner::apply(const Eigen::VectorXd& r) const {
    return ilu_.solve(r);
}

} // namespace fem::linalg
