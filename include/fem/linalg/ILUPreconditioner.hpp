/// @file ILUPreconditioner.hpp
/// @brief Incomplete-LU Preconditioner implementation (CPU-only).
#pragma once
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseLU>
#include "fem/linalg/Preconditioner.hpp"
#include "fem/linalg/EigenSparseOperator.hpp"

namespace fem::linalg {

/// @brief Incomplete-LU preconditioner wrapping Eigen::IncompleteLUT.
///
/// Step 2.2. CPU-only by construction (needs the full matrix to factor,
/// not just applyTo) — see ARCHITECTURE.md §7 for the two options
/// (widen LinearOperator vs. scope Jacobi/ILU to CPU) and pick one before
/// Step 3.3.
class ILUPreconditioner : public Preconditioner {
public:
    /// @copydoc Preconditioner::setup
    /// @throws std::logic_error if op is not backed by an EigenSparseOperator
    /// (ILU needs the full matrix, not just applyTo — see class docs).
    void setup(const LinearOperator& op) override;

    /// @copydoc Preconditioner::apply
    Eigen::VectorXd apply(const Eigen::VectorXd& r) const override;

private:
    Eigen::IncompleteLUT<double> ilu_;   ///< Eigen's incomplete-LU factorization, rebuilt each setup() call.
};

} // namespace fem::linalg
