/// @file SimulationConfig.hpp
/// @brief Plain data structures describing one simulation, parsed from an
/// input file. Deliberately dependency-free — no JSON types leak out of
/// io::loadSimulationConfig, so nothing outside src/fem/io/InputReader.cpp
/// needs to know or care which file format is used underneath.
#pragma once
#include <array>
#include <map>
#include <string>
#include <vector>

namespace fem::io {

/// @brief Structured-mesh generation parameters.
struct MeshConfig {
    std::string type = "structured_cube";      ///< "structured_cube" or "structured_cylinder".
    std::array<int, 3> elementCounts{1, 1, 1};  ///< Cube: x/y/z counts; cylinder: radial/angular/axial counts.
    std::array<double, 3> elementSize{1.0, 1.0, 1.0}; ///< Cube cell sizes along x/y/z (unused for cylinder).
    std::string elementType = "hex8";          ///< "hex8" or "tet4" — selects Element subclass.
    double innerRadius = 1.0;                   ///< Inner radius for a structured cylinder sector.
    double outerRadius = 2.0;                   ///< Outer radius for a structured cylinder sector.
    double axialLength = 1.0;                   ///< Axial length for a structured cylinder sector.
};

/// @brief Material model selection and its named parameters.
struct MaterialConfig {
    std::string type;                    ///< e.g. "neo-hookean", "mooney-rivlin".
    std::map<std::string, double> params; ///< Named parameters, e.g. {"mu": 1.0, "kappa": 10.0}; GDB pretty-printers show entries instead of tree internals.
};

/// @brief One boundary condition, expressed in terms of a mesh face rather
/// than raw DOF indices — MeshBuilder resolves "face" to actual node ids.
struct BoundaryConditionConfig {
    std::string type = "dirichlet";      ///< "dirichlet" or follower "pressure".
    std::string face;                    ///< Mesh face key (cube or cylinder face).
    std::array<double, 3> value{0.0, 0.0, 0.0}; ///< Prescribed displacement (dirichlet) per axis.
    std::array<bool, 3> components{true, true, true}; ///< Dirichlet components; existing inputs constrain all axes.
    double pressure = 0.0;               ///< Positive pressure magnitude for pressure BCs.
};

/// @brief Linear-solve strategy selection for each Newton iteration.
struct SolverConfig {
    std::string linearSolver = "direct";     ///< "direct", "cg", or "gmres".
    std::string preconditioner = "identity"; ///< "identity", "jacobi", or "ilu".
    double tolerance = 1e-8;                 ///< Iterative solver convergence tolerance.
    int maxIterations = 1000;                ///< Iterative solver iteration cap.
};

/// @brief Newton-Raphson load-stepping parameters.
struct NewtonConfig {
    int loadSteps = 1;                     ///< Number of load increments.
    double residualTolerance = 1e-6;       ///< Convergence tolerance on residual norm.
    double displacementTolerance = 1e-6;   ///< Convergence tolerance on displacement increment norm.
    int maxIterationsPerStep = 10;         ///< Newton iteration cap per load step.
};

/// @brief The full description of one simulation, as read from an input file.
struct SimulationConfig {
    MeshConfig mesh;
    MaterialConfig material;
    std::vector<BoundaryConditionConfig> boundaryConditions;
    SolverConfig solver;
    NewtonConfig newton;
};

} // namespace fem::io
