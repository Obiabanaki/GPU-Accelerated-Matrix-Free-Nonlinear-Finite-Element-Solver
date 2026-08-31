/// @file IdentityPreconditioner.hpp
/// @brief No-op Preconditioner implementation (M = I).
#pragma once
#include "fem/linalg/Preconditioner.hpp"

namespace fem::linalg {

/// @brief No-op preconditioner: z = r. Default for ConjugateGradientSolver.
class IdentityPreconditioner : public Preconditioner {
public:
    /// @copydoc Preconditioner::setup
    void setup(const LinearOperator& op) override;

    /// @copydoc Preconditioner::apply
    Eigen::VectorXd apply(const Eigen::VectorXd& r) const override;
};

} // namespace fem::linalg
