/// @file DirichletBC.cpp
/// @brief Implementation of DirichletBC. TRACE MODE.
#include "fem/bc/DirichletBC.hpp"
#include <iostream>
#include <stdexcept>

namespace fem {

DirichletBC::DirichletBC(std::vector<int> dofs, std::vector<double> prescribedValues)
    : dofs_(std::move(dofs)), values_(std::move(prescribedValues)) {
    if (dofs_.size() != values_.size()) {
        throw std::invalid_argument("DirichletBC: dofs and prescribedValues must have equal length");
    }
    std::cout << "[DirichletBC] constructed on " << dofs_.size() << " dofs\n";
}

void DirichletBC::apply(GlobalSystem& system, const Eigen::VectorXd& displacement,
                        double loadFactor) const {
    if (displacement.size() != system.numDofs()) {
        throw std::invalid_argument("DirichletBC::apply: displacement size does not match system");
    }

    auto& K = system.tangent();
    auto& rhs = system.residual();
    for (std::size_t constraint = 0; constraint < dofs_.size(); ++constraint) {
        const int dof = dofs_[constraint];
        if (dof < 0 || dof >= system.numDofs()) {
            throw std::out_of_range("DirichletBC::apply: constrained DOF is outside the system");
        }
        const double increment = loadFactor * values_[constraint] - displacement[dof];

        // Eliminate both the constrained column and row. The column contribution
        // is transferred to the free-DOF right-hand side to preserve symmetry.
        for (int column = 0; column < K.outerSize(); ++column) {
            for (Eigen::SparseMatrix<double>::InnerIterator entry(K, column); entry; ++entry) {
                if (entry.col() == dof) {
                    if (entry.row() != dof) rhs[entry.row()] -= entry.value() * increment;
                    entry.valueRef() = 0.0;
                } else if (entry.row() == dof) {
                    entry.valueRef() = 0.0;
                }
            }
        }
        K.coeffRef(dof, dof) = 1.0;
        rhs[dof] = increment;
    }
    K.prune([](int, int, double value) { return value != 0.0; });
    K.makeCompressed();
}

} // namespace fem
