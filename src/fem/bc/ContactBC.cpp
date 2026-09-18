/// @file ContactBC.cpp
/// @brief Implementation of ContactBC. TRACE MODE.
#include "fem/bc/ContactBC.hpp"
#include <iostream>

namespace fem {

ContactBC::ContactBC(std::vector<int> candidateNodeDofs, Eigen::Vector3d planeNormal,
                      double planeOffset, double penaltyStiffness)
    : candidateDofs_(std::move(candidateNodeDofs)), planeNormal_(planeNormal),
      planeOffset_(planeOffset), penalty_(penaltyStiffness) {
    std::cout << "[ContactBC] constructed with " << candidateDofs_.size()
              << " candidate dofs, penalty=" << penalty_ << "\n";
}

void ContactBC::apply(GlobalSystem& system) const {
    std::cout << "[ContactBC::apply] would check " << candidateDofs_.size()
              << " candidate nodes for penetration and add penalty contributions (trace mode)\n";
    (void)system;
}

} // namespace fem
