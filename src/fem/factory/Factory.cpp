/// @file Factory.cpp
/// @brief Implementation of the createMaterial/createLinearSolver factory functions.
#include "fem/factory/Factory.hpp"
#include "fem/material/NeoHookeanMaterial.hpp"
#include "fem/material/MooneyRivlinMaterial.hpp"
#include "fem/linalg/DirectSolver.hpp"
#include "fem/linalg/ConjugateGradientSolver.hpp"
#include "fem/linalg/GMRESSolver.hpp"
#include <stdexcept>

namespace fem::factory {

std::unique_ptr<fem::Material> createMaterial(const std::string& type,
                                               const std::map<std::string, double>& params) {
    if (type == "neo-hookean") {
        return std::make_unique<fem::NeoHookeanMaterial>(params.at("mu"), params.at("kappa"));
    }
    if (type == "mooney-rivlin") {
        return std::make_unique<fem::MooneyRivlinMaterial>(
            params.at("c10"), params.at("c01"), params.at("kappa"));
    }
    throw std::invalid_argument("createMaterial: unknown type '" + type + "'");
}

std::unique_ptr<fem::linalg::LinearSolver> createLinearSolver(
    const std::string& type, fem::linalg::Preconditioner& preconditioner) {
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
