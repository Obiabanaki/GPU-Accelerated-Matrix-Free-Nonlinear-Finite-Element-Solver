/// @file DeviceCsrMatrix.cpp
/// @brief Implementation of DeviceCsrMatrix.
#include "fem/backend/DeviceCsrMatrix.hpp"

namespace fem::backend {

DeviceCsrMatrix DeviceCsrMatrix::fromEigen(const Eigen::SparseMatrix<double>& A) {
    // TODO(Step 3.3): extract A.valuePtr()/innerIndexPtr()/outerIndexPtr()
    // into this object's CSR arrays.
    DeviceCsrMatrix m;
    m.numRows_ = static_cast<int>(A.rows());
    return m;
}

} // namespace fem::backend
