/// @file test_linear_operator.cpp
/// @brief Verifies the LinearOperator seam: EigenSparseOperator's applyTo
/// must match plain Eigen sparse multiplication, and (once implemented)
/// BackendOperator's applyTo must match EigenSparseOperator's to tolerance
/// for the same underlying matrix (Step 3.3 acceptance criterion).
#include <gtest/gtest.h>
#include <Eigen/Sparse>
#include "fem/linalg/EigenSparseOperator.hpp"

TEST(EigenSparseOperator, ApplyToMatchesDirectMultiplication) {
    Eigen::SparseMatrix<double> A(3, 3);
    A.insert(0, 0) = 2.0;
    A.insert(1, 1) = 3.0;
    A.insert(2, 2) = 4.0;
    A.makeCompressed();

    fem::linalg::EigenSparseOperator op(A);
    Eigen::VectorXd x(3);
    x << 1.0, 1.0, 1.0;

    EXPECT_TRUE(op.applyTo(x).isApprox(A * x));
}

TEST(EigenSparseOperator, SizeMatchesMatrixDimension) {
    Eigen::SparseMatrix<double> A(5, 5);
    fem::linalg::EigenSparseOperator op(A);
    EXPECT_EQ(op.size(), 5);
}
