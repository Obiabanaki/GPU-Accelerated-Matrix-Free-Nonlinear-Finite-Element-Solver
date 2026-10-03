/// @file NewtonSolver.cpp
/// @brief Nonlinear equilibrium solve using assembled internal forces and tangent stiffness.
#include "fem/solver/NewtonSolver.hpp"
#include "fem/linalg/EigenSparseOperator.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

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
    if (numLoadSteps <= 0 || maxIterPerStep <= 0 || !std::isfinite(residualTol) ||
        !std::isfinite(dispTol) || residualTol <= 0.0 || dispTol <= 0.0) {
        throw std::invalid_argument("NewtonSolver::solve: steps, iteration limit, and tolerances must be positive");
    }
    std::cout << "\n[NewtonSolver::solve] starting: " << numLoadSteps << " load step(s), "
              << "residualTol=" << residualTol << ", dispTol=" << dispTol
              << ", maxIterPerStep=" << maxIterPerStep << "\n";

    Eigen::VectorXd u = Eigen::VectorXd::Zero(mesh_.numDofs());
    GlobalSystem system(mesh_.numDofs());
    convergenceHistory_.clear();
    double acceptedLoadFactor = 0.0;

    for (int step = 1; step <= numLoadSteps; ++step) {
        std::cout << "\n-- Load step " << step << "/" << numLoadSteps << " --\n";
        convergenceHistory_.emplace_back();

        const double targetLoadFactor = static_cast<double>(step) / numLoadSteps;
        double nextIncrement = targetLoadFactor - acceptedLoadFactor;
        int consecutiveCutbacks = 0;
        while (acceptedLoadFactor < targetLoadFactor) {
            const double attemptedIncrement = nextIncrement;
            const double trialLoadFactor = std::min(
                targetLoadFactor, acceptedLoadFactor + attemptedIncrement);
            const Eigen::VectorXd acceptedDisplacement = u;
            bool converged = false;

            for (int iter = 1; iter <= maxIterPerStep; ++iter) {
                std::cout << "  Newton iteration " << iter
                          << " (load factor " << trialLoadFactor << "):\n";

                system.reset();
                mesh_.assemble(system, material_, u);
                system.finalize();
                system.residual() *= -1.0;
                for (auto& bc : boundaryConditions_) {
                    bc.get().apply(system, u, trialLoadFactor);
                }

                const double residualNorm = system.residual().norm();
                if (!std::isfinite(residualNorm)) {
                    throw std::runtime_error("NewtonSolver::solve: non-finite residual encountered");
                }

                linalg::EigenSparseOperator op(system.tangent());
                Eigen::VectorXd du = linearSolver_.solve(op, system.residual());
                if (du.size() != u.size() || !du.allFinite()) {
                    throw std::runtime_error("NewtonSolver::solve: linear solver returned an invalid increment");
                }
                const double displacementIncrementNorm = du.norm();
                convergenceHistory_.back().push_back(residualNorm);
                std::cout << "    residual norm = " << residualNorm
                          << ", displacement increment norm = " << displacementIncrementNorm << "\n";

                if (residualNorm <= residualTol && displacementIncrementNorm <= dispTol) {
                    converged = true;
                    break;
                }
                u += du;
            }

            if (converged) {
                acceptedLoadFactor = trialLoadFactor;
                nextIncrement = std::min(targetLoadFactor - acceptedLoadFactor,
                                         2.0 * attemptedIncrement);
                consecutiveCutbacks = 0;
                continue;
            }

            u = acceptedDisplacement;
            ++consecutiveCutbacks;
            if (consecutiveCutbacks > 12 || attemptedIncrement <= 1e-10 / numLoadSteps) {
                throw std::runtime_error("NewtonSolver::solve: load step " + std::to_string(step) +
                                         " failed after load-increment cutbacks");
            }
            nextIncrement = 0.5 * attemptedIncrement;
            std::cout << "  Cutting back load increment to " << nextIncrement << "\n";
        }
    }

    std::cout << "\n[NewtonSolver::solve] finished all " << numLoadSteps << " load step(s)\n";
    return u;
}

} // namespace fem
