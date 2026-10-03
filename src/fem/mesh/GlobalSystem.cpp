/// @file GlobalSystem.cpp
/// @brief Implementation of global residual and sparse tangent assembly.
#include "fem/mesh/GlobalSystem.hpp"
#include <iostream>
#include <stdexcept>

namespace fem {

GlobalSystem::GlobalSystem(int numDofs)
    : numDofs_(numDofs), residual_(Eigen::VectorXd::Zero(numDofs)), K_(numDofs, numDofs) {}

void GlobalSystem::reset() {
    residual_.setZero();
    triplets_.clear();
    K_.setZero();
}

void GlobalSystem::addResidual(const std::vector<int>& dofs, const Eigen::VectorXd& localR) {
    if (localR.size() != static_cast<Eigen::Index>(dofs.size())) {
        throw std::invalid_argument("GlobalSystem::addResidual: local residual size does not match DOFs");
    }
    for (std::size_t local = 0; local < dofs.size(); ++local) {
        if (dofs[local] < 0 || dofs[local] >= numDofs_) {
            throw std::out_of_range("GlobalSystem::addResidual: global DOF is outside the system");
        }
        residual_[dofs[local]] += localR[static_cast<Eigen::Index>(local)];
    }
}

void GlobalSystem::addTangent(const std::vector<int>& dofs, const Eigen::MatrixXd& localK) {
    const auto localDofCount = static_cast<Eigen::Index>(dofs.size());
    if (localK.rows() != localDofCount || localK.cols() != localDofCount) {
        throw std::invalid_argument("GlobalSystem::addTangent: local tangent dimensions do not match DOFs");
    }
    for (int dof : dofs) {
        if (dof < 0 || dof >= numDofs_) {
            throw std::out_of_range("GlobalSystem::addTangent: global DOF is outside the system");
        }
    }
    for (Eigen::Index row = 0; row < localDofCount; ++row) {
        for (Eigen::Index column = 0; column < localDofCount; ++column) {
            triplets_.emplace_back(dofs[static_cast<std::size_t>(row)],
                                   dofs[static_cast<std::size_t>(column)], localK(row, column));
        }
    }
}

void GlobalSystem::finalize() {
    K_.setFromTriplets(triplets_.begin(), triplets_.end());
    K_.makeCompressed();
}

Eigen::SparseMatrix<double>& GlobalSystem::tangent() { return K_; }
const Eigen::SparseMatrix<double>& GlobalSystem::tangent() const { return K_; }
Eigen::VectorXd& GlobalSystem::residual() { return residual_; }
const Eigen::VectorXd& GlobalSystem::residual() const { return residual_; }

} // namespace fem
