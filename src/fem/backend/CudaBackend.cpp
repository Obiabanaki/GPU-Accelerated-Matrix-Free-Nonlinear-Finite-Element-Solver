/// @file CudaBackend.cpp
/// @brief Implementation of CudaBackend.
#include "fem/backend/CudaBackend.hpp"

namespace fem::backend {

Eigen::VectorXd CudaBackend::spmv(const DeviceCsrMatrix& A, const Eigen::VectorXd& x) const {
    // TODO(Step 3.3): one thread per row CSR SpMV kernel; validate against
    // CpuBackend::spmv on the Phase 1 benchmark problem to tolerance.
    // TODO(Step 3.4): memory coalescing (CSR reordering or warp-per-row).
    (void)A;
    return Eigen::VectorXd::Zero(x.size());
}

} // namespace fem::backend
