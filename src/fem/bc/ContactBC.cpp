/// @file ContactBC.cpp
/// @brief Implementation of ContactBC.
#include "fem/bc/ContactBC.hpp"

namespace fem {

ContactBC::ContactBC(std::vector<int> candidateNodeDofs, Eigen::Vector3d planeNormal,
                      double planeOffset, double penaltyStiffness)
    : candidateDofs_(std::move(candidateNodeDofs)), planeNormal_(planeNormal),
      planeOffset_(planeOffset), penalty_(penaltyStiffness) {}

void ContactBC::apply(GlobalSystem& system) const {
    // TODO(Step 2.3-stretch): for each candidate node, compute gap = 
    // dot(position, planeNormal_) - planeOffset_; if gap < 0, add
    // penalty_ * gap * planeNormal_ contributions to residual and tangent.
    (void)system;
}

} // namespace fem
