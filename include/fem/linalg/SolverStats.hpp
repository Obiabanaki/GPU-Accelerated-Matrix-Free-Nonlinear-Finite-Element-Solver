/// @file SolverStats.hpp
/// @brief Diagnostics struct returned by LinearSolver::lastSolveStats().
#pragma once
#include <vector>

namespace fem::linalg {

/// @brief Diagnostics from a LinearSolver::solve() call. Trivial/empty for
/// DirectSolver, populated for iterative solvers (Step 2.2's benchmark
/// deliverable — iteration count and convergence history — reads this).
struct SolverStats {
    int iterations = 0;                    ///< Number of iterations performed (0 for direct solves).
    std::vector<double> residualHistory;   ///< Residual norm after each iteration, in order.
};

} // namespace fem::linalg
