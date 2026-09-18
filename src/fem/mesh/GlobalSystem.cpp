/// @file GlobalSystem.cpp
/// @brief Implementation of GlobalSystem.
/// TRACE MODE: reset/tangent()/residual()/numDofs() stay real (trivial
/// bookkeeping/accessors); addResidual/addTangent/finalize only trace.
#include "fem/mesh/GlobalSystem.hpp"
#include <iostream>

namespace fem {

GlobalSystem::GlobalSystem(int numDofs)
    : numDofs_(numDofs), residual_(Eigen::VectorXd::Zero(numDofs)), K_(numDofs, numDofs) {}

void GlobalSystem::reset() {
    residual_.setZero();
    triplets_.clear();
}

void GlobalSystem::addResidual(const std::vector<int>& dofs, const Eigen::VectorXd& localR) {
    std::cout << "[GlobalSystem::addResidual] would scatter a " << localR.size()
              << "-entry local residual into " << dofs.size() << " global dofs (trace mode)\n";
}

void GlobalSystem::addTangent(const std::vector<int>& dofs, const Eigen::MatrixXd& localK) {
    std::cout << "[GlobalSystem::addTangent] would push " << localK.rows() * localK.cols()
              << " triplet entries for " << dofs.size() << " dofs (trace mode)\n";
}

void GlobalSystem::finalize() {
    K_.setFromTriplets(triplets_.begin(), triplets_.end()); // no-op: triplets_ stays empty in trace mode
}

Eigen::SparseMatrix<double>& GlobalSystem::tangent() { return K_; }
const Eigen::SparseMatrix<double>& GlobalSystem::tangent() const { return K_; }
Eigen::VectorXd& GlobalSystem::residual() { return residual_; }
const Eigen::VectorXd& GlobalSystem::residual() const { return residual_; }

} // namespace fem
