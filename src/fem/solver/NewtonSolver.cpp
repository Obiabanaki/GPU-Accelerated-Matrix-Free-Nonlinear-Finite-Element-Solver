/// @file NewtonSolver.cpp
/// @brief Newton solve loop for the current facade pass.
///
/// The outer load-step / Newton-iteration loop is real: it genuinely iterates
/// over the requested steps, invokes mesh_.assemble(), applies each boundary
/// condition, solves the linearized system, and advances the displacement
/// iterate in the same order a real Newton method would use. The collaborators
/// themselves are still in trace mode, so no actual constitutive update or
/// linear-algebra solve is performed yet. This class is therefore a real
/// orchestration layer, not a physically complete solver.
///
/// The convergence check below is intentionally a placeholder for the later
/// residual/displacement-norm logic and step-halving cutback logic.
#include "fem/solver/NewtonSolver.hpp"
#include "fem/linalg/EigenSparseOperator.hpp"
#include <iostream>

namespace fem {

NewtonSolver::NewtonSolver(Mesh& mesh, Material& material, linalg::LinearSolver& linearSolver,
                            std::vector<std::reference_wrapper<BoundaryCondition>> boundaryConditions)
    : mesh_(mesh), material_(material), linearSolver_(linearSolver),
      boundaryConditions_(std::move(boundaryConditions)) {
    std::cout << "[NewtonSolver] constructed with " << boundaryConditions_.size()
              << " boundary condition(s)\n";
}

Eigen::VectorXd NewtonSolver::solve(int numLoadSteps, double residualTol, double dispTol,
                                     int maxIterPerStep) {
    std::cout << "\n[NewtonSolver::solve] starting: " << numLoadSteps << " load step(s), "
              << "residualTol=" << residualTol << ", dispTol=" << dispTol
              << ", maxIterPerStep=" << maxIterPerStep << "\n";

    Eigen::VectorXd u = Eigen::VectorXd::Zero(mesh_.numDofs());
    GlobalSystem system(mesh_.numDofs());
    convergenceHistory_.clear();

    for (int step = 1; step <= numLoadSteps; ++step) {
        std::cout << "\n-- Load step " << step << "/" << numLoadSteps << " --\n";
        convergenceHistory_.emplace_back();

        for (int iter = 1; iter <= maxIterPerStep; ++iter) {
            std::cout << "  Newton iteration " << iter << ":\n";

            system.reset();
            mesh_.assemble(system, material_, u);
            for (auto& bc : boundaryConditions_) {
                bc.get().apply(system);
            }
            system.finalize();

            linalg::EigenSparseOperator op(system.tangent());
            Eigen::VectorXd du = linearSolver_.solve(op, system.residual());
            u += du;

            // Placeholder convergence check — see file comment. Real logic
            // (residual-norm AND displacement-increment-norm checks, with
            // step-halving cutback on non-convergence) is Step 1.4 scope.
            const double placeholderResidualNorm = 0.0;
            convergenceHistory_.back().push_back(placeholderResidualNorm);
            std::cout << "    residual norm (placeholder) = " << placeholderResidualNorm
                      << " -> treating as converged (trace mode)\n";
            break;
        }
    }

    std::cout << "\n[NewtonSolver::solve] finished all " << numLoadSteps << " load step(s)\n";
    return u;
}

} // namespace fem
