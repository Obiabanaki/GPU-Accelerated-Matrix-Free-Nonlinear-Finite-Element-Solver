/// @file main.cpp
/// @brief Composition root for a configured nonlinear FEM simulation.
///
/// Reads the JSON input, builds the configured mesh/material/boundary
/// conditions/linear solver, and runs NewtonSolver. The Hex8, material,
/// assembly, Dirichlet, and direct-solver path computes a displacement field;
/// trace-only alternatives such as Tet4 physics and iterative solvers remain
/// incomplete.
///
/// Usage: fem_demo <path-to-input.json>
/// e.g.:  ./fem_demo examples/example_problem.json
#include <iostream>
#include <vector>
#include "fem/io/InputReader.hpp"
#include "fem/mesh/MeshBuilder.hpp"
#include "fem/factory/Factory.hpp"
#include "fem/solver/NewtonSolver.hpp"

namespace {

/// @brief Build a Dirichlet or pressure condition from named mesh-face data.
std::unique_ptr<fem::BoundaryCondition> buildBoundaryCondition(
    const fem::io::BoundaryConditionConfig& bcConfig,
    const fem::mesh::BuiltMesh& built) {
    if (bcConfig.type == "pressure") {
        const auto facetsIt = built.faceQuadNodeIds.find(bcConfig.face);
        if (facetsIt == built.faceQuadNodeIds.end()) {
            throw std::invalid_argument("buildBoundaryCondition: face has no pressure facets '" +
                                        bcConfig.face + "'");
        }
        std::vector<fem::PressureFacet> facets;
        for (const auto& nodeIds : facetsIt->second) {
            fem::PressureFacet facet;
            facet.nodeIds = nodeIds;
            for (int node = 0; node < 4; ++node) {
                facet.referenceCoordinates[node] = built.mesh->nodeCoordinates()[nodeIds[node]];
            }
            facets.push_back(std::move(facet));
        }
        return fem::factory::createPressureBoundaryCondition(std::move(facets), bcConfig.pressure);
    }

    const auto it = built.faceNodeIds.find(bcConfig.face);
    if (it == built.faceNodeIds.end()) {
        throw std::invalid_argument("buildBoundaryCondition: unknown face '" + bcConfig.face + "'");
    }

    std::vector<int> dofs;
    std::vector<double> values;
    for (int nodeId : it->second) {
        for (int component = 0; component < 3; ++component) {
            if (bcConfig.components[component]) {
                dofs.push_back(3 * nodeId + component);
                values.push_back(bcConfig.value[component]);
            }
        }
    }
    return fem::factory::createBoundaryCondition(bcConfig.type, std::move(dofs), std::move(values));
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path-to-input.json>\n"
                  << "e.g.:  " << argv[0] << " examples/example_problem.json\n";
        return 1;
    }

    try {
        std::cout << "=== nonlinear-fem-solver ===\n";
        std::cout << "Reading input file: " << argv[1] << "\n";
        fem::io::SimulationConfig config = fem::io::loadSimulationConfig(argv[1]);

        // --- Mesh ---
        fem::mesh::BuiltMesh built = fem::mesh::buildMesh(config.mesh);
        // For debugging, set a breakpoint on the next statement: built.mesh is
        // a unique_ptr, so inspect the Mesh through built.mesh->... .

        // --- Material ---
        auto material = fem::factory::createMaterial(config.material.type, config.material.params);

        // --- Boundary conditions ---
        // Keep each boundary condition alive for the whole solve; NewtonSolver
        // refers to these objects but does not own them.
        std::vector<std::unique_ptr<fem::BoundaryCondition>> bcOwners;
        for (const auto& bcConfig : config.boundaryConditions) {
            bcOwners.push_back(buildBoundaryCondition(bcConfig, built));
        }
        // Build the reference list expected by NewtonSolver without copying or
        // transferring ownership of the boundary conditions.
        std::vector<std::reference_wrapper<fem::BoundaryCondition>> bcRefs;
        for (auto& bc : bcOwners) {
            bcRefs.push_back(*bc);
        }

        // --- Linear solver + preconditioner ---
        auto preconditioner = fem::factory::createPreconditioner(config.solver.preconditioner);
        auto linearSolver = fem::factory::createLinearSolver(config.solver.linearSolver, *preconditioner);

        // --- Newton solver, wired to everything above via abstract references only ---
        fem::NewtonSolver newtonSolver(*built.mesh, *material, *linearSolver, bcRefs);
        const Eigen::VectorXd displacement = newtonSolver.solve(
            config.newton.loadSteps, config.newton.residualTolerance,
            config.newton.displacementTolerance, config.newton.maxIterationsPerStep);

        std::cout << "\nConverged displacement vector (node-major x/y/z DOFs):\n"
                  << displacement.transpose() << "\n";
        std::cout << "\n=== Simulation converged ===\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
