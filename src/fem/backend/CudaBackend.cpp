/// @file CudaBackend.cpp
/// @brief Implementation of CudaBackend. TRACE MODE.
#include "fem/backend/CudaBackend.hpp"
#include <iostream>

namespace fem::backend {

Eigen::VectorXd CudaBackend::spmv(const DeviceCsrMatrix& A, const Eigen::VectorXd& x) const {
    std::cout << "[CudaBackend::spmv] would launch a CSR SpMV kernel for a "
              << A.numRows() << "-row matrix (trace mode)\n";
    return Eigen::VectorXd::Zero(x.size());
}

} // namespace fem::backend
