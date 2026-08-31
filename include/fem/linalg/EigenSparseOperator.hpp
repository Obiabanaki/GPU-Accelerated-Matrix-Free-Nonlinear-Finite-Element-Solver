/// @file EigenSparseOperator.hpp
/// @brief CPU-resident LinearOperator implementation wrapping Eigen::SparseMatrix.
#pragma once
#include <Eigen/Sparse>
#include "fem/linalg/LinearOperator.hpp"

namespace fem::linalg {

/// @brief Wraps a CPU-resident Eigen sparse matrix.
///
/// Used by DirectSolver always, and by ConjugateGradientSolver/GMRESSolver
/// through Phase 2 (CPU-only). Constructed fresh each Newton iteration
/// from GlobalSystem::tangent() — a thin, temporary view, not a long-lived
/// owner. Must not outlive the GlobalSystem it wraps.
class EigenSparseOperator : public LinearOperator {
public:
    /// @brief Wrap an existing CPU sparse matrix by reference.
    /// @param A Matrix to wrap; must outlive this operator.
    explicit EigenSparseOperator(const Eigen::SparseMatrix<double>& A);

    /// @copydoc LinearOperator::applyTo
    Eigen::VectorXd applyTo(const Eigen::VectorXd& x) const override;

    /// @copydoc LinearOperator::size
    int size() const override;

    /// @copydoc LinearOperator::diagonal
    Eigen::VectorXd diagonal() const override;

private:
    const Eigen::SparseMatrix<double>& A_;   ///< Wrapped matrix; not owned, must outlive this operator.
};

} // namespace fem::linalg
