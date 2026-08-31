/// @file IdentityPreconditioner.cpp
/// @brief Implementation of IdentityPreconditioner.
#include "fem/linalg/IdentityPreconditioner.hpp"

namespace fem::linalg {

void IdentityPreconditioner::setup(const LinearOperator& op) { (void)op; }

Eigen::VectorXd IdentityPreconditioner::apply(const Eigen::VectorXd& r) const { return r; }

} // namespace fem::linalg
