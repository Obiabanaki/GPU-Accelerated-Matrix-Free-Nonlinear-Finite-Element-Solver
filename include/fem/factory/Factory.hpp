/// @file Factory.hpp
/// @brief Composition-root factory functions (createMaterial, createLinearSolver).
#pragma once
#include <map>
#include <memory>
#include <string>
#include "fem/material/Material.hpp"
#include "fem/linalg/LinearSolver.hpp"
#include "fem/linalg/Preconditioner.hpp"

namespace fem::factory {

/// @brief Constructs a concrete Material from a name and parameter map.
///
/// This — together with createLinearSolver below — is meant to be the ONLY
/// place in the codebase (outside each class's own translation unit and
/// tests) permitted to name a concrete class like NeoHookeanMaterial or
/// ConjugateGradientSolver. Grep the rest of the codebase for concrete
/// type names as the project-wide version of the "is the Strategy pattern
/// real" check from ARCHITECTURE.md.
/// @param type Material name: "neo-hookean" or "mooney-rivlin".
/// @param params Named parameter values required by the chosen type (e.g.
/// "mu"/"kappa" for neo-hookean; "c10"/"c01"/"kappa" for mooney-rivlin).
/// @return Newly constructed Material, owned by the caller.
/// @throws std::invalid_argument if type is not recognized.
std::unique_ptr<fem::Material> createMaterial(const std::string& type,
                                               const std::map<std::string, double>& params);

/// @brief Constructs a concrete LinearSolver from a name.
/// @param type Solver name: "direct", "cg", or "gmres".
/// @param preconditioner Only used by iterative solvers (CG, GMRES);
/// ignored for "direct". Must outlive the returned solver.
/// @return Newly constructed LinearSolver, owned by the caller.
/// @throws std::invalid_argument if type is not recognized.
std::unique_ptr<fem::linalg::LinearSolver> createLinearSolver(
    const std::string& type, fem::linalg::Preconditioner& preconditioner);

} // namespace fem::factory
