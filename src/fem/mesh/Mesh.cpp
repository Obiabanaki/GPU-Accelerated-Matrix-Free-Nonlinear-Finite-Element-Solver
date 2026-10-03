/// @file Mesh.cpp
/// @brief Mesh assembly facade for the current trace pass.
///
/// Mesh::assemble genuinely iterates over every element and invokes each
/// concrete Element's virtual residual/tangent methods in the correct order.
/// The global-system scatter and the bookkeeping needed to populate the final
/// residual/tangent entries are still intentionally deferred in this pass, so
/// the method logs the orchestration path rather than computing a physically
/// final assembled system.
#include "fem/mesh/Mesh.hpp"
#include <iostream>
#include <stdexcept>

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
    if (globalDisplacement.size() != numDofs() || system.numDofs() != numDofs()) {
        throw std::invalid_argument("Mesh::assemble: displacement and system sizes must match mesh DOFs");
    }

    // Assemble each element's local force and stiffness into global DOF order.
    for (const auto& element : elements_) {
        const auto& ids = element->nodeIds();
        std::vector<int> elementDofs;
        elementDofs.reserve(ids.size() * 3);
        Eigen::VectorXd localDisplacement(static_cast<Eigen::Index>(ids.size() * 3));
        for (std::size_t localNode = 0; localNode < ids.size(); ++localNode) {
            const int nodeId = ids[localNode];
            if (nodeId < 0 || nodeId >= static_cast<int>(dofMap_.size())) {
                throw std::out_of_range("Mesh::assemble: element references an unknown node");
            }
            for (int component = 0; component < 3; ++component) {
                const int globalDof = dofMap_[nodeId][component];
                elementDofs.push_back(globalDof);
                localDisplacement[static_cast<Eigen::Index>(localNode * 3 + component)] =
                    globalDisplacement[globalDof];
            }
        }

        const Eigen::VectorXd localResidual = element->computeResidual(localDisplacement, material);
        const Eigen::MatrixXd localTangent = element->computeTangentStiffness(localDisplacement, material);
        system.addResidual(elementDofs, localResidual);
        system.addTangent(elementDofs, localTangent);
    }
}

} // namespace fem
