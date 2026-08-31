/// @file CudaBackend.hpp
/// @brief CUDA-accelerated ComputeBackend implementation (Phase 3).
#pragma once
#include "fem/backend/ComputeBackend.hpp"

namespace fem::backend {

/// @brief CUDA-accelerated SpMV. Phase 3, Step 3.3.
///
/// Only compiled when FEM_ENABLE_CUDA is ON (see root CMakeLists.txt) —
/// GPU testing stays local, per the project plan; hosted CI runners don't
/// have GPUs, so Phases 0-2 correctness is CI's scope, not this class.
///
/// CRITICAL: must cache the uploaded matrix across calls within one CG
/// solve (only x changes between CG iterations, not A) or Step 3.4's
/// benchmark will show no speedup for the wrong reason — see
/// ARCHITECTURE.md's CudaBackend design check.
class CudaBackend : public ComputeBackend {
public:
    /// @copydoc ComputeBackend::spmv
    Eigen::VectorXd spmv(const DeviceCsrMatrix& A, const Eigen::VectorXd& x) const override;

private:
    // TODO(Step 3.3): device-side cache — e.g. track the DeviceCsrMatrix
    // pointer/generation last uploaded, skip re-upload if unchanged.
};

} // namespace fem::backend
