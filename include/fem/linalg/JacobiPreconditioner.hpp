/// @file JacobiPreconditioner.hpp
/// @brief Diagonal (Jacobi) Preconditioner implementation.
#pragma once
#include "fem/linalg/Preconditioner.hpp"

namespace fem::linalg {

/// @brief Diagonal (Jacobi) preconditioner: z_i = r_i / A_ii.
///
/// Step 2.2. Requires LinearOperator::diagonal() — see the matrix-free
/// tension noted in ARCHITECTURE.md §7 and Preconditioner.hpp.
class JacobiPreconditioner : public Preconditioner {
public:
    /// @copydoc Preconditioner::setup
    void setup(const LinearOperator& op) override;

    /// @copydoc Preconditioner::apply
    Eigen::VectorXd apply(const Eigen::VectorXd& r) const override;

private:
    Eigen::VectorXd invDiag_;   ///< Cached elementwise inverse of A's diagonal, rebuilt each setup() call.
};

} // namespace fem::linalg
