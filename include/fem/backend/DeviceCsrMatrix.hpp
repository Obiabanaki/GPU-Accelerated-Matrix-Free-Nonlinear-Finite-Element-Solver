/// @file DeviceCsrMatrix.hpp
/// @brief Backend-resident CSR matrix handle (DeviceCsrMatrix).
#pragma once
#include <vector>
#include <Eigen/Sparse>

namespace fem::backend {

/// @brief CSR matrix arrays, resident wherever ComputeBackend put them
/// (host memory for CpuBackend, device memory for CudaBackend).
///
/// This is the "backend-resident matrix" that BackendOperator wraps. The
/// point of caching it (rather than re-deriving it from Eigen every call)
/// is to avoid re-uploading to the GPU on every CG iteration — see
/// ARCHITECTURE.md's CudaBackend design check.
class DeviceCsrMatrix {
public:
    /// @brief Build/upload from a CPU Eigen sparse matrix. For CpuBackend
    /// this is just a view; for CudaBackend this triggers a one-time
    /// host->device transfer.
    /// @param A Source matrix in CPU/Eigen sparse format.
    /// @return A new DeviceCsrMatrix resident on whichever backend calls this.
    static DeviceCsrMatrix fromEigen(const Eigen::SparseMatrix<double>& A);

    /// @brief Number of rows (= number of columns; the matrix is square).
    /// @return Row count.
    int numRows() const { return numRows_; }

    // TODO(Step 3.3): CSR arrays (values, colIndices, rowPtr), either as
    // std::vector<double>/<int> (CpuBackend) or device pointers (CudaBackend).
private:
    int numRows_ = 0;   ///< Matrix row/column count.
};

} // namespace fem::backend
