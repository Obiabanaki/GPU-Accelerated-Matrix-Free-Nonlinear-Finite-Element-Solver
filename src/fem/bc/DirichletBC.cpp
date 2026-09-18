/// @file DirichletBC.cpp
/// @brief Implementation of DirichletBC. TRACE MODE.
#include "fem/bc/DirichletBC.hpp"
#include <iostream>

namespace fem {

DirichletBC::DirichletBC(std::vector<int> dofs, std::vector<double> prescribedValues)
    : dofs_(std::move(dofs)), values_(std::move(prescribedValues)) {
    std::cout << "[DirichletBC] constructed on " << dofs_.size() << " dofs\n";
}

void DirichletBC::apply(GlobalSystem& system) const {
    std::cout << "[DirichletBC::apply] would eliminate " << dofs_.size()
              << " rows/columns via row/column elimination (trace mode)\n";
    (void)system;
}

} // namespace fem
