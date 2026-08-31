/// @file LinearOperator.cpp
/// @brief Implementation of LinearOperator's default (throwing) diagonal().
#include "fem/linalg/LinearOperator.hpp"
#include <stdexcept>

namespace fem::linalg {

Eigen::VectorXd LinearOperator::diagonal() const {
    throw std::logic_error(
        "LinearOperator::diagonal() not implemented for this operator. "
        "Either this operator needs to override it (see ARCHITECTURE.md §7), "
        "or the preconditioner requesting it should be scoped out for this backend.");
}

} // namespace fem::linalg
