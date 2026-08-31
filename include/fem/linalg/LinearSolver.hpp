/// @file LinearSolver.hpp
/// @brief Abstract LinearSolver strategy interface — see ARCHITECTURE.md §6b.
#pragma once
#include <Eigen/Dense>
#include "fem/linalg/LinearOperator.hpp"
#include "fem/linalg/SolverStats.hpp"

namespace fem::linalg {

/// @brief Strategy interface for solving A*du = R, where A is only ever
/// accessed through the abstract LinearOperator interface.
///
/// NewtonSolver calls solve(op, R) and never knows or cares whether op
/// wraps a CPU matrix or GPU-resident data, or whether the solve is a
/// dense LU factorization or 500 iterations of preconditioned CG.
class LinearSolver {
public:
    /// @brief Virtual destructor; LinearSolver is always used polymorphically.
    virtual ~LinearSolver() = default;

    /// @brief Solve the linear system for the Newton increment.
    /// @param op Abstract operator representing the tangent matrix.
    /// @param R Residual vector (right-hand side).
    /// @return du, the Newton displacement increment.
    virtual Eigen::VectorXd solve(LinearOperator& op, const Eigen::VectorXd& R) = 0;

    /// @brief Diagnostics from the most recent solve() call.
    /// @return Iteration count and residual history (empty/trivial for direct solvers).
    virtual SolverStats lastSolveStats() const = 0;
};

} // namespace fem::linalg
