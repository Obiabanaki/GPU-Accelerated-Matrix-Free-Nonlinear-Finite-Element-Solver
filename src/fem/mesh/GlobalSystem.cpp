/// @file GlobalSystem.cpp
/// @brief Implementation of GlobalSystem.
#include "fem/mesh/GlobalSystem.hpp"

namespace fem {

GlobalSystem::GlobalSystem(int numDofs)
    : numDofs_(numDofs), residual_(Eigen::VectorXd::Zero(numDofs)), K_(numDofs, numDofs) {}

void GlobalSystem::reset() {
    residual_.setZero();
    triplets_.clear();
    // K_ is rebuilt from triplets_ in finalize(); no need to zero it here.
}

void GlobalSystem::addResidual(const std::vector<int>& dofs, const Eigen::VectorXd& localR) {
    // TODO(Step 1.3): scatter-add localR into residual_ at global dofs.
    (void)dofs; (void)localR;
}

void GlobalSystem::addTangent(const std::vector<int>& dofs, const Eigen::MatrixXd& localK) {
    // TODO(Step 1.3): push Eigen::Triplet<double> entries for every
    // (dofs[i], dofs[j], localK(i,j)) pair. Deliberately buffered, not
    // written directly into K_ — see class doc comment for why.
    (void)dofs; (void)localK;
}

void GlobalSystem::finalize() {
    K_.setFromTriplets(triplets_.begin(), triplets_.end());
}

Eigen::SparseMatrix<double>& GlobalSystem::tangent() { return K_; }
const Eigen::SparseMatrix<double>& GlobalSystem::tangent() const { return K_; }
Eigen::VectorXd& GlobalSystem::residual() { return residual_; }
const Eigen::VectorXd& GlobalSystem::residual() const { return residual_; }

} // namespace fem
