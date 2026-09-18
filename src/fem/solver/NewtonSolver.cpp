/// @file NewtonSolver.cpp
/// @brief Implementation of NewtonSolver.
///
/// TRACE MODE, but of a different kind than Material/Element/LinearSolver:
/// the outer load-step / Newton-iteration LOOP STRUCTURE below is real —
/// it genuinely loops numLoadSteps times and, within each, genuinely calls
/// mesh_.assemble, each boundary condition's apply(), and linearSolver_.solve
/// in the real order a working Newton solver would. Those collaborators are
/// themselves in trace mode (see their own file comments), so no actual
/// linear algebra happens — but the *procedure* — the sequence of object
/// interactions this class is responsible for orchestrating — is real and
/// observable in the printed trace, which is the point of this facade pass.
///
/// The "convergence" check below always succeeds after one iteration; it's
/// a placeholder for Step 1.4's real residual/displacement-norm check and
/// step-halving cutback logic, not an implementation of them.
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
