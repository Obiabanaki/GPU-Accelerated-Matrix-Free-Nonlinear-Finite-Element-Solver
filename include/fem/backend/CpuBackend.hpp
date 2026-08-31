/// @file CpuBackend.hpp
/// @brief Trivial CPU ComputeBackend implementation.
#pragma once
#include "fem/backend/ComputeBackend.hpp"

namespace fem::backend {

/// @brief Trivial CPU SpMV. Confirms ConjugateGradientSolver was written
/// against LinearOperator/ComputeBackend, never against a raw matrix —
/// this class exists mainly as that proof, and as the CI-testable
/// baseline CudaBackend's output is checked against.
class CpuBackend : public ComputeBackend {
public:
    /// @copydoc ComputeBackend::spmv
    Eigen::VectorXd spmv(const DeviceCsrMatrix& A, const Eigen::VectorXd& x) const override;
};

} // namespace fem::backend
