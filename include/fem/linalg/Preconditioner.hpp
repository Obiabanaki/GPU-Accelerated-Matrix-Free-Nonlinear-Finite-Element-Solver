/// @file Preconditioner.hpp
/// @brief Abstract Preconditioner strategy interface — see ARCHITECTURE.md §7.
#pragma once
#include <Eigen/Dense>
#include "fem/linalg/LinearOperator.hpp"

namespace fem::linalg {

/// @brief Strategy interface: approximately solve M*z = r for M ~= A.
///
/// setup() is separate from apply() deliberately: A changes every Newton
/// iteration, so the preconditioner must be rebuilt every iteration too —
/// but setup() must be called once per Newton iteration, NOT once per CG
/// iteration. See ARCHITECTURE.md §7 for the diagonal()/matrix-free gap
/// this interface works around.
class Preconditioner {
public:
    /// @brief Virtual destructor; Preconditioner is always used polymorphically.
    virtual ~Preconditioner() = default;

    /// @brief Build/update internal state from the current operator.
    /// @param op Current-iteration tangent operator to precondition against.
    virtual void setup(const LinearOperator& op) = 0;

    /// @brief Apply the preconditioner: z = M^{-1} r.
    /// @param r Residual vector to precondition, length matching the operator size.
    /// @return z = M^{-1} r.
    virtual Eigen::VectorXd apply(const Eigen::VectorXd& r) const = 0;
};

} // namespace fem::linalg
