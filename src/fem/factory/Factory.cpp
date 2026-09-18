/// @file Factory.cpp
/// @brief Implementation of the composition-root factory functions.
#include "fem/factory/Factory.hpp"
#include "fem/material/NeoHookeanMaterial.hpp"
#include "fem/material/MooneyRivlinMaterial.hpp"
#include "fem/element/Hex8Element.hpp"
#include "fem/element/Tet4Element.hpp"
#include "fem/bc/DirichletBC.hpp"
#include "fem/linalg/DirectSolver.hpp"
#include "fem/linalg/ConjugateGradientSolver.hpp"
#include "fem/linalg/GMRESSolver.hpp"
#include "fem/linalg/IdentityPreconditioner.hpp"
#include "fem/linalg/JacobiPreconditioner.hpp"
#include "fem/linalg/ILUPreconditioner.hpp"
#include <iostream>
#include <stdexcept>

namespace fem::factory {

std::unique_ptr<fem::Material> createMaterial(const std::string& type,
                                               const std::map<std::string, double>& params) {
    if (type == "neo-hookean") {
        std::cout << "[Factory] creating NeoHookeanMaterial(mu=" << params.at("mu")
                  << ", kappa=" << params.at("kappa") << ")\n";
        return std::make_unique<fem::NeoHookeanMaterial>(params.at("mu"), params.at("kappa"));
    }
    if (type == "mooney-rivlin") {
        std::cout << "[Factory] creating MooneyRivlinMaterial(c10=" << params.at("c10")
                  << ", c01=" << params.at("c01") << ", kappa=" << params.at("kappa") << ")\n";
        return std::make_unique<fem::MooneyRivlinMaterial>(
            params.at("c10"), params.at("c01"), params.at("kappa"));
    }
    throw std::invalid_argument("createMaterial: unknown type '" + type + "'");
}

std::unique_ptr<fem::Element> createElement(const std::string& type,
                                             std::vector<int> nodeIds,
                                             std::vector<Eigen::Vector3d> nodeCoords) {
    if (type == "hex8") {
        if (nodeIds.size() != 8) {
            throw std::invalid_argument("createElement: hex8 requires exactly 8 nodes");
        }
        std::array<int, 8> ids;
        std::array<Eigen::Vector3d, 8> coords;
        std::copy(nodeIds.begin(), nodeIds.end(), ids.begin());
        std::copy(nodeCoords.begin(), nodeCoords.end(), coords.begin());
        return std::make_unique<fem::Hex8Element>(ids, coords);
    }
    if (type == "tet4") {
        if (nodeIds.size() != 4) {
            throw std::invalid_argument("createElement: tet4 requires exactly 4 nodes");
        }
        std::array<int, 4> ids;
        std::array<Eigen::Vector3d, 4> coords;
        std::copy(nodeIds.begin(), nodeIds.end(), ids.begin());
        std::copy(nodeCoords.begin(), nodeCoords.end(), coords.begin());
        return std::make_unique<fem::Tet4Element>(ids, coords);
    }
    throw std::invalid_argument("createElement: unknown type '" + type + "'");
}

std::unique_ptr<fem::BoundaryCondition> createBoundaryCondition(
    const std::string& type, std::vector<int> dofs, std::vector<double> values) {
    if (type == "dirichlet") {
        std::cout << "[Factory] creating DirichletBC on " << dofs.size() << " dofs\n";
        return std::make_unique<fem::DirichletBC>(std::move(dofs), std::move(values));
    }
    throw std::invalid_argument("createBoundaryCondition: unknown type '" + type + "'");
}

std::unique_ptr<fem::linalg::Preconditioner> createPreconditioner(const std::string& type) {
    std::cout << "[Factory] creating " << type << " preconditioner\n";
    if (type == "identity") return std::make_unique<fem::linalg::IdentityPreconditioner>();
    if (type == "jacobi") return std::make_unique<fem::linalg::JacobiPreconditioner>();
    if (type == "ilu") return std::make_unique<fem::linalg::ILUPreconditioner>();
    throw std::invalid_argument("createPreconditioner: unknown type '" + type + "'");
}

std::unique_ptr<fem::linalg::LinearSolver> createLinearSolver(
    const std::string& type, fem::linalg::Preconditioner& preconditioner) {
    std::cout << "[Factory] creating " << type << " linear solver\n";
    if (type == "direct") {
        return std::make_unique<fem::linalg::DirectSolver>();
    }
    if (type == "cg") {
        return std::make_unique<fem::linalg::ConjugateGradientSolver>(preconditioner);
    }
    if (type == "gmres") {
        return std::make_unique<fem::linalg::GMRESSolver>(preconditioner);
    }
    throw std::invalid_argument("createLinearSolver: unknown type '" + type + "'");
}

} // namespace fem::factory
