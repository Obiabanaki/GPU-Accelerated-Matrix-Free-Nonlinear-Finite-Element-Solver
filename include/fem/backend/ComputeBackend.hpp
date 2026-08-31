/// @file ComputeBackend.hpp
/// @brief Abstract ComputeBackend strategy interface (SpMV execution site).
#pragma once
#include <Eigen/Dense>
#include "fem/backend/DeviceCsrMatrix.hpp"

namespace fem::backend {

/// @brief Strategy interface isolating *where* a sparse mat-vec product runs.
///
/// ConjugateGradientSolver calls backend.spmv(...) INDIRECTLY, through
/// linalg::BackendOperator — never directly. That's what makes GPU support
/// an additive CudaBackend sibling, not a rewrite of CG. The only class
/// permitted to hold a ComputeBackend& is BackendOperator.
class ComputeBackend {
public:
    /// @brief Virtual destructor; ComputeBackend is always used polymorphically.
    virtual ~ComputeBackend() = default;

    /// @brief Compute y = A * x.
    /// @param A Backend-resident matrix to multiply.
    /// @param x Input vector, length A.numRows().
    /// @return y = A * x, length A.numRows().
    virtual Eigen::VectorXd spmv(const DeviceCsrMatrix& A, const Eigen::VectorXd& x) const = 0;
};

} // namespace fem::backend
