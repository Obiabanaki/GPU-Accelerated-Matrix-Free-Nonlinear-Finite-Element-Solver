/// @file InputReader.hpp
/// @brief Parses a simulation input file into a SimulationConfig.
#pragma once
#include <string>
#include "fem/io/SimulationConfig.hpp"

namespace fem::io {

/// @brief Load and parse a JSON simulation input file.
/// @param path Path to the input file (e.g. "examples/example_problem.json").
/// @return Fully populated SimulationConfig.
/// @throws std::runtime_error if the file can't be opened or parsed, or a
/// required field is missing.
SimulationConfig loadSimulationConfig(const std::string& path);

} // namespace fem::io
