/// @file Mesh.cpp
/// @brief Implementation of Mesh.
#include "fem/mesh/Mesh.hpp"

namespace fem {

void Mesh::addElement(std::unique_ptr<Element> element) {
    elements_.push_back(std::move(element));
}

void Mesh::assemble(GlobalSystem& system, const Material& material,
                     const Eigen::VectorXd& globalDisplacement) const {
    // TODO(Step 1.3): for each element in elements_, gather its local
    // displacement slice from globalDisplacement via dofMap_, call
    // element->computeResidual/computeTangentStiffness, and scatter the
    // results into `system` via addResidual/addTangent. This loop is
    // polymorphic over Element and must stay that way — no concrete
    // element-type checks here.
    (void)system; (void)material; (void)globalDisplacement;
}

} // namespace fem
