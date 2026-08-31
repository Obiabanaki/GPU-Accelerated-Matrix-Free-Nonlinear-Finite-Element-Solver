/// @file BackendOperator.hpp
/// @brief GPU-resident LinearOperator implementation wrapping ComputeBackend.
#pragma once
#include <Eigen/Dense>
#include "fem/linalg/LinearOperator.hpp"
#include "fem/backend/ComputeBackend.hpp"
#include "fem/backend/DeviceCsrMatrix.hpp"

namespace fem::linalg {

/// @brief Wraps a backend-resident matrix (Phase 3 onward).
///
/// All device residency and upload-caching logic lives here and in
/// ComputeBackend — ConjugateGradientSolver never knows this class exists,
/// only that it received a LinearOperator&.
class BackendOperator : public LinearOperator {
public:
    /// @brief Wrap an existing backend-resident matrix by reference.
    /// @param backend Compute backend that will perform the SpMV (e.g. CudaBackend).
    /// @param A Backend-resident matrix; must outlive this operator.
    BackendOperator(backend::ComputeBackend& backend, const backend::DeviceCsrMatrix& A);

    /// @copydoc LinearOperator::applyTo
    Eigen::VectorXd applyTo(const Eigen::VectorXd& x) const override;

    /// @copydoc LinearOperator::size
    int size() const override;

    /// @copydoc LinearOperator::diagonal
    Eigen::VectorXd diagonal() const override;

private:
    backend::ComputeBackend& backend_;         ///< Backend used to perform spmv(); not owned.
    const backend::DeviceCsrMatrix& A_;        ///< Backend-resident matrix, already uploaded; not owned.
};

} // namespace fem::linalg
