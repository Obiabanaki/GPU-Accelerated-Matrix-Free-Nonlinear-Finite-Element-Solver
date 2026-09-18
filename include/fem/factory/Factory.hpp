/// @file Factory.hpp
/// @brief Composition-root factory functions — the only place in the
/// codebase (outside each class's own translation unit and tests)
/// permitted to name a concrete class. Grep the rest of the codebase for
/// concrete type names as the project-wide "is the Strategy pattern real"
/// check from ARCHITECTURE.md.
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "fem/material/Material.hpp"
#include "fem/element/Element.hpp"
#include "fem/bc/BoundaryCondition.hpp"
#include "fem/linalg/LinearSolver.hpp"
#include "fem/linalg/Preconditioner.hpp"

namespace fem::factory {

/// @brief Constructs a concrete Material from a name and parameter map.
/// @param type Material name: "neo-hookean" or "mooney-rivlin".
/// @param params Named parameter values required by the chosen type (e.g.
/// "mu"/"kappa" for neo-hookean; "c10"/"c01"/"kappa" for mooney-rivlin).
/// @return Newly constructed Material, owned by the caller.
/// @throws std::invalid_argument if type is not recognized.
std::unique_ptr<fem::Material> createMaterial(const std::string& type,
                                               const std::map<std::string, double>& params);

/// @brief Constructs a concrete Element from a name and its geometry.
/// @param type Element name: "hex8" (needs 8 nodes) or "tet4" (needs 4 nodes).
/// @param nodeIds Global node indices, in the concrete class's expected order.
/// @param nodeCoords Reference-configuration coordinates, matching nodeIds.
/// @return Newly constructed Element, owned by the caller.
/// @throws std::invalid_argument if type is not recognized or the node
/// count doesn't match what the type requires.
std::unique_ptr<fem::Element> createElement(const std::string& type,
                                             std::vector<int> nodeIds,
                                             std::vector<Eigen::Vector3d> nodeCoords);

/// @brief Constructs a concrete BoundaryCondition from primitive DOF data.
/// @param type BC name: "dirichlet" (more types added as they're implemented).
/// @param dofs Global DOF indices affected.
/// @param values Prescribed value per dof — same length and order as dofs.
/// @return Newly constructed BoundaryCondition, owned by the caller.
/// @throws std::invalid_argument if type is not recognized.
std::unique_ptr<fem::BoundaryCondition> createBoundaryCondition(
    const std::string& type, std::vector<int> dofs, std::vector<double> values);

/// @brief Constructs a concrete Preconditioner from a name.
/// @param type Preconditioner name: "identity", "jacobi", or "ilu".
/// @return Newly constructed Preconditioner, owned by the caller.
/// @throws std::invalid_argument if type is not recognized.
std::unique_ptr<fem::linalg::Preconditioner> createPreconditioner(const std::string& type);

/// @brief Constructs a concrete LinearSolver from a name.
/// @param type Solver name: "direct", "cg", or "gmres".
/// @param preconditioner Only used by iterative solvers (CG, GMRES);
/// ignored for "direct". Must outlive the returned solver.
/// @return Newly constructed LinearSolver, owned by the caller.
/// @throws std::invalid_argument if type is not recognized.
std::unique_ptr<fem::linalg::LinearSolver> createLinearSolver(
    const std::string& type, fem::linalg::Preconditioner& preconditioner);

} // namespace fem::factory
