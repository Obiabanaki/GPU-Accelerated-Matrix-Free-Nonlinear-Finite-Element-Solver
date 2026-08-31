/// @file DirichletBC.cpp
/// @brief Implementation of DirichletBC.
#include "fem/bc/DirichletBC.hpp"

namespace fem {

DirichletBC::DirichletBC(std::vector<int> dofs, std::vector<double> prescribedValues)
    : dofs_(std::move(dofs)), values_(std::move(prescribedValues)) {}

void DirichletBC::apply(GlobalSystem& system) const {
    // TODO(Step 1.4): standard row/column elimination — for each dof in
    // dofs_: zero its row/column in system.tangent() except a 1 on the
    // diagonal, and set system.residual()[dof] to the (incremental)
    // prescribed displacement minus current displacement at that dof.
    (void)system;
}

} // namespace fem
