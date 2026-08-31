/// @file LinearOperator.hpp
/// @brief The CPU/GPU seam abstraction (LinearOperator) — see ARCHITECTURE.md §6a.
#pragma once
#include <Eigen/Dense>

namespace fem::linalg {

/// @brief Abstract "A*x without materializing A" operator.
///
/// This is the seam between assembly (always CPU/Eigen, via GlobalSystem)
/// and linear solve (may be CPU or GPU). LinearSolver and Preconditioner
/// depend only on this interface, never on a concrete matrix type or on
/// backend::ComputeBackend — that's what makes "swap CpuBackend ->
/// CudaBackend with zero changes to CG" an actual, checkable claim.
class LinearOperator {
public:
    /// @brief Virtual destructor; LinearOperator is always used polymorphically.
    virtual ~LinearOperator() = default;

    /// @brief Compute y = A * x without ever exposing A itself.
    /// @param x Input vector, length size().
    /// @return y = A * x, length size().
    virtual Eigen::VectorXd applyTo(const Eigen::VectorXd& x) const = 0;

    /// @brief Matrix dimension (needed by CG/GMRES to size work vectors).
    /// @return Number of rows (= number of columns; A is square).
    virtual int size() const = 0;

    /// @brief Optional: diagonal of A, if cheaply available. Needed by
    /// JacobiPreconditioner. Default throws — see ARCHITECTURE.md §7 for
    /// the preconditioner/matrix-free tension this resolves.
    /// @return Vector of diagonal entries of A, length size().
    /// @throws std::logic_error if the concrete operator hasn't overridden this.
    virtual Eigen::VectorXd diagonal() const;
};

} // namespace fem::linalg
