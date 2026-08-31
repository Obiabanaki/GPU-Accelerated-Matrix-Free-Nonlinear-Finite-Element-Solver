/// @file BackendOperator.cpp
/// @brief Implementation of BackendOperator.
#include "fem/linalg/BackendOperator.hpp"

namespace fem::linalg {

BackendOperator::BackendOperator(backend::ComputeBackend& backend,
                                  const backend::DeviceCsrMatrix& A)
    : backend_(backend), A_(A) {}

Eigen::VectorXd BackendOperator::applyTo(const Eigen::VectorXd& x) const {
    return backend_.spmv(A_, x);
}

int BackendOperator::size() const {
    return A_.numRows();
}

Eigen::VectorXd BackendOperator::diagonal() const {
    // TODO(Step 3.3, if the diagonal()-escape-hatch option is chosen):
    // a fast device-side gather of A_'s diagonal, not a full SpMV.
    return LinearOperator::diagonal(); // throws by default — see base class
}

} // namespace fem::linalg
