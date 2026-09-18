/// @file IdentityPreconditioner.cpp
/// @brief Implementation of IdentityPreconditioner.
/// Genuinely trivial even outside trace mode: z = r has no "real
/// implementation" to defer, so this one isn't changed by trace mode.
#include "fem/linalg/IdentityPreconditioner.hpp"
#include <iostream>

namespace fem::linalg {

void IdentityPreconditioner::setup(const LinearOperator& op) {
    std::cout << "[IdentityPreconditioner::setup] no-op by design\n";
    (void)op;
}

Eigen::VectorXd IdentityPreconditioner::apply(const Eigen::VectorXd& r) const { return r; }

} // namespace fem::linalg
