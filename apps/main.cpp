/// @file main.cpp
/// @brief Composition root for the current facade/tracing pass.
///
/// This application genuinely reads the JSON input, resolves mesh and face
/// node metadata, instantiates the configured concrete classes, and drives the
/// Newton solve loop. The orchestration layer is real; the remaining FEM math
/// in the current pass is intentionally trace-oriented, so the program logs the
/// intended sequence of object interactions without computing a final deformed
/// shape.
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

/// @brief Resolve one BoundaryConditionConfig (face + prescribed value)
/// into DOF indices and per-dof prescribed values, then hand off to the
/// factory. Kept here rather than in Factory itself, so Factory stays
/// decoupled from the io::SimulationConfig representation.
std::unique_ptr<fem::BoundaryCondition> buildBoundaryCondition(
    const fem::io::BoundaryConditionConfig& bcConfig,
    const std::map<std::string, std::vector<int>>& faceNodeIds) {
    auto it = faceNodeIds.find(bcConfig.face);
    if (it == faceNodeIds.end()) {
        throw std::invalid_argument("buildBoundaryCondition: unknown face '" + bcConfig.face + "'");
    }

    std::vector<int> dofs;
    std::vector<double> values;
    for (int nodeId : it->second) {
        for (int component = 0; component < 3; ++component) {
            dofs.push_back(3 * nodeId + component);
            values.push_back(bcConfig.value[component]);
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
        std::cout << "=== nonlinear-fem-solver (facade / tracing mode) ===\n";
        std::cout << "Reading input file: " << argv[1] << "\n";
        fem::io::SimulationConfig config = fem::io::loadSimulationConfig(argv[1]);

        // --- Mesh ---
        fem::mesh::BuiltMesh built = fem::mesh::buildStructuredCubeMesh(config.mesh);

        // --- Material ---
        auto material = fem::factory::createMaterial(config.material.type, config.material.params);

        // --- Boundary conditions ---
        std::vector<std::unique_ptr<fem::BoundaryCondition>> bcOwners;
        for (const auto& bcConfig : config.boundaryConditions) {
            bcOwners.push_back(buildBoundaryCondition(bcConfig, built.faceNodeIds));
        }
        std::vector<std::reference_wrapper<fem::BoundaryCondition>> bcRefs;
        for (auto& bc : bcOwners) {
            bcRefs.push_back(*bc);
        }

        // --- Linear solver + preconditioner ---
        auto preconditioner = fem::factory::createPreconditioner(config.solver.preconditioner);
        auto linearSolver = fem::factory::createLinearSolver(config.solver.linearSolver, *preconditioner);

        // --- Newton solver, wired to everything above via abstract references only ---
        fem::NewtonSolver newtonSolver(*built.mesh, *material, *linearSolver, bcRefs);
        newtonSolver.solve(config.newton.loadSteps, config.newton.residualTolerance,
                            config.newton.displacementTolerance, config.newton.maxIterationsPerStep);

        std::cout << "\n=== Simulation finished (facade mode — no physical results were computed) ===\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
