/// @file CpuBackend.cpp
/// @brief Implementation of CpuBackend. TRACE MODE.
#include "fem/backend/CpuBackend.hpp"
#include <iostream>

namespace fem::backend {

Eigen::VectorXd CpuBackend::spmv(const DeviceCsrMatrix& A, const Eigen::VectorXd& x) const {
    std::cout << "[CpuBackend::spmv] would compute y = A*x on CPU for a "
              << A.numRows() << "-row matrix (trace mode)\n";
    return Eigen::VectorXd::Zero(x.size());
}

} // namespace fem::backend
