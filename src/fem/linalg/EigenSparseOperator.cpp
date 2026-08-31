/// @file EigenSparseOperator.cpp
/// @brief Implementation of EigenSparseOperator.
#include "fem/linalg/EigenSparseOperator.hpp"

namespace fem::linalg {

EigenSparseOperator::EigenSparseOperator(const Eigen::SparseMatrix<double>& A) : A_(A) {}

Eigen::VectorXd EigenSparseOperator::applyTo(const Eigen::VectorXd& x) const {
    return A_ * x;
}

int EigenSparseOperator::size() const {
    return static_cast<int>(A_.rows());
}

Eigen::VectorXd EigenSparseOperator::diagonal() const {
    return A_.diagonal();
}

} // namespace fem::linalg
