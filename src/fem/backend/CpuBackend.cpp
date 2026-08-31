/// @file CpuBackend.cpp
/// @brief Implementation of CpuBackend.
#include "fem/backend/CpuBackend.hpp"

namespace fem::backend {

Eigen::VectorXd CpuBackend::spmv(const DeviceCsrMatrix& A, const Eigen::VectorXd& x) const {
    // TODO(Step 3.3): straightforward CSR SpMV loop (or, more simply,
    // reconstruct/hold an Eigen::SparseMatrix view and use Eigen's own
    // multiply — CpuBackend's whole point is being the trivial baseline).
    (void)A;
    return Eigen::VectorXd::Zero(x.size());
}

} // namespace fem::backend
