/// @file test_newton_solver.cpp
/// @brief Design check + behavioral tests for NewtonSolver, including the
/// Step 1.4 requirement to demonstrate cutback firing and recovering.
#include <gtest/gtest.h>

TEST(NewtonSolver, ConvergesQuadraticallyNearSolution) {
    // TODO(Step 1.4): log(residual) vs iteration should show quadratic
    // convergence once the solver reaches Phase 1's benchmark problem.
    GTEST_SKIP() << "TODO: implement once Mesh/NewtonSolver are filled in";
}

TEST(NewtonSolver, CutbackFiresAndRecoversOnOverlargeLoadStep) {
    // TODO(Step 1.4): deliberately over-large load step should trigger
    // step-halving cutback and still reach a converged solution.
    GTEST_SKIP() << "TODO: implement once cutback logic is filled in";
}
