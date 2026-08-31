/// @file main.cpp
/// @brief Composition root: the only place concrete classes are wired
/// together outside their own translation units and the factory.
#include <iostream>
#include "fem/factory/Factory.hpp"
#include "fem/linalg/IdentityPreconditioner.hpp"

int main() {
    // TODO: build a Mesh, wire up createMaterial/createLinearSolver,
    // construct NewtonSolver, run solve(), write VTK output.
    auto material = fem::factory::createMaterial("neo-hookean", {{"mu", 1.0}, {"kappa", 10.0}});
    fem::linalg::IdentityPreconditioner precond;
    auto solver = fem::factory::createLinearSolver("direct", precond);

    std::cout << "nonlinear-fem-solver skeleton — material and solver constructed OK.\n";
    return 0;
}
