/// @file InputReader.cpp
/// @brief Implementation of loadSimulationConfig using nlohmann::json.
///
/// nlohmann::json is intentionally confined to this one translation unit —
/// SimulationConfig.hpp has no knowledge of it, so nothing outside this
/// file (and fem_core's PRIVATE link to nlohmann_json in src/CMakeLists.txt)
/// needs to know which JSON library, or even that JSON specifically, is
/// used to read the input file.
#include "fem/io/InputReader.hpp"
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace fem::io {

namespace {

using json = nlohmann::json;

MeshConfig parseMesh(const json& j) {
    MeshConfig cfg;
    cfg.type = j.value("type", cfg.type);
    if (j.contains("element_counts")) {
        auto ec = j.at("element_counts");
        cfg.elementCounts = {ec.at(0).get<int>(), ec.at(1).get<int>(), ec.at(2).get<int>()};
    }
    if (j.contains("element_size")) {
        auto es = j.at("element_size");
        cfg.elementSize = {es.at(0).get<double>(), es.at(1).get<double>(), es.at(2).get<double>()};
    }
    cfg.elementType = j.value("element_type", cfg.elementType);
    return cfg;
}

MaterialConfig parseMaterial(const json& j) {
    MaterialConfig cfg;
    cfg.type = j.at("type").get<std::string>();
    if (j.contains("params")) {
        for (auto& [key, value] : j.at("params").items()) {
            cfg.params[key] = value.get<double>();
        }
    }
    return cfg;
}

std::vector<BoundaryConditionConfig> parseBoundaryConditions(const json& j) {
    std::vector<BoundaryConditionConfig> result;
    for (const auto& bcJson : j) {
        BoundaryConditionConfig bc;
        bc.type = bcJson.value("type", bc.type);
        bc.face = bcJson.at("face").get<std::string>();
        if (bcJson.contains("value")) {
            auto v = bcJson.at("value");
            bc.value = {v.at(0).get<double>(), v.at(1).get<double>(), v.at(2).get<double>()};
        }
        result.push_back(bc);
    }
    return result;
}

SolverConfig parseSolver(const json& j) {
    SolverConfig cfg;
    cfg.linearSolver = j.value("linear_solver", cfg.linearSolver);
    cfg.preconditioner = j.value("preconditioner", cfg.preconditioner);
    cfg.tolerance = j.value("tolerance", cfg.tolerance);
    cfg.maxIterations = j.value("max_iterations", cfg.maxIterations);
    return cfg;
}

NewtonConfig parseNewton(const json& j) {
    NewtonConfig cfg;
    cfg.loadSteps = j.value("load_steps", cfg.loadSteps);
    cfg.residualTolerance = j.value("residual_tolerance", cfg.residualTolerance);
    cfg.displacementTolerance = j.value("displacement_tolerance", cfg.displacementTolerance);
    cfg.maxIterationsPerStep = j.value("max_iterations_per_step", cfg.maxIterationsPerStep);
    return cfg;
}

} // namespace

SimulationConfig loadSimulationConfig(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("loadSimulationConfig: could not open '" + path + "'");
    }

    json root;
    try {
        file >> root;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("loadSimulationConfig: JSON parse error in '" + path + "': " + e.what());
    }

    SimulationConfig config;
    try {
        config.mesh = parseMesh(root.at("mesh"));
        config.material = parseMaterial(root.at("material"));
        if (root.contains("boundary_conditions")) {
            config.boundaryConditions = parseBoundaryConditions(root.at("boundary_conditions"));
        }
        config.solver = parseSolver(root.at("solver"));
        config.newton = parseNewton(root.at("newton"));
    } catch (const json::out_of_range& e) {
        throw std::runtime_error("loadSimulationConfig: missing required field in '" + path + "': " + e.what());
    }

    return config;
}

} // namespace fem::io
