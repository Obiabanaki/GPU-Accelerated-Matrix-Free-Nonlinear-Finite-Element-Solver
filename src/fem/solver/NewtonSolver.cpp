/// @file NewtonSolver.cpp
/// @brief Implementation of NewtonSolver.
#include "fem/solver/NewtonSolver.hpp"
#include "fem/linalg/EigenSparseOperator.hpp"

namespace fem {

NewtonSolver::NewtonSolver(Mesh& mesh, Material& material, linalg::LinearSolver& linearSolver,
                            std::vector<std::reference_wrapper<BoundaryCondition>> boundaryConditions)
    : mesh_(mesh), material_(material), linearSolver_(linearSolver),
      boundaryConditions_(std::move(boundaryConditions)) {}

Eigen::VectorXd NewtonSolver::solve(int numLoadSteps, double residualTol, double dispTol,
                                     int maxIterPerStep) {
    // TODO(Step 1.4): full load-stepped Newton loop with cutback. Skeleton
    // of one iteration, showing the one place EigenSparseOperator gets
    // constructed:
    //
    // GlobalSystem system(mesh_.numDofs());
    // for (int step = 0; step < numLoadSteps; ++step) {
    //     for (int iter = 0; iter < maxIterPerStep; ++iter) {
    //         system.reset();
    //         mesh_.assemble(system, material_, u);
    //         for (auto& bc : boundaryConditions_) bc.get().apply(system);
    //         system.finalize();
    //
    //         linalg::EigenSparseOperator op(system.tangent());
    //         Eigen::VectorXd du = linearSolver_.solve(op, system.residual());
    //         u += du;
    //
    //         convergenceHistory_.back().push_back(system.residual().norm());
    //         if (converged) break;
    //     }
    //     // on non-convergence: halve step size, retry from last converged
    //     // state, down to a configurable minimum before declaring failure.
    // }
    (void)numLoadSteps; (void)residualTol; (void)dispTol; (void)maxIterPerStep;
    return Eigen::VectorXd::Zero(mesh_.numDofs());
}

} // namespace fem
