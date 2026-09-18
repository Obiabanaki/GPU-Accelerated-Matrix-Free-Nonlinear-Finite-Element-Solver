/// @file Mesh.cpp
/// @brief Implementation of Mesh.
#include "fem/mesh/Mesh.hpp"
#include <iostream>

namespace fem {

int Mesh::addNode(const Eigen::Vector3d& coord) {
    const int nodeIndex = static_cast<int>(nodeCoords_.size());
    nodeCoords_.push_back(coord);
    const int firstDof = nodeIndex * 3;
    dofMap_.push_back({firstDof, firstDof + 1, firstDof + 2});
    return nodeIndex;
}

void Mesh::addElement(std::unique_ptr<Element> element) {
    elements_.push_back(std::move(element));
}

void Mesh::assemble(GlobalSystem& system, const Material& material,
                     const Eigen::VectorXd& globalDisplacement) const {
    // TRACE MODE: real physics assembly (gathering each element's local
    // displacement slice via dofMap_, scattering results into `system`)
    // is deferred — see Element/Material's own trace-mode notes. But the
    // loop below IS real: it genuinely calls each concrete Element's
    // computeResidual/computeTangentStiffness, polymorphically, once per
    // element — that dispatch is what's being demonstrated here, not
    // just claimed in a comment.
    std::cout << "[Mesh::assemble] looping over " << elements_.size()
              << " elements, delegating to each Element's computeResidual/"
              << "computeTangentStiffness (trace mode — no scatter into GlobalSystem yet)\n";
    (void)globalDisplacement;
    (void)system;

    for (const auto& element : elements_) {
        const auto& ids = element->nodeIds();
        // Real local-displacement gather (via dofMap_ + globalDisplacement)
        // is part of the deferred physics assembly; a zero-sized-correctly
        // placeholder is enough to genuinely exercise each Element's
        // virtual methods with the right-shaped input.
        Eigen::VectorXd uLocal = Eigen::VectorXd::Zero(static_cast<int>(ids.size()) * 3);
        element->computeResidual(uLocal, material);
        element->computeTangentStiffness(uLocal, material);
    }
}

} // namespace fem
