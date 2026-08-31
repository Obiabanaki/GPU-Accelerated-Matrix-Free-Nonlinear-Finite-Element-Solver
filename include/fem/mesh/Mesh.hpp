/// @file Mesh.hpp
/// @brief Mesh topology owner and assembly driver — see ARCHITECTURE.md §4.
#pragma once
#include <memory>
#include <vector>
#include <array>
#include "fem/element/Element.hpp"
#include "fem/mesh/GlobalSystem.hpp"

namespace fem {

/// @brief Owns mesh topology (nodes, elements, DOF map) and drives assembly.
///
/// Mesh contains no physics: it loops over Elements and delegates all
/// mechanics to them polymorphically. See ARCHITECTURE.md's open question
/// on single-material-per-mesh vs. per-element before relying on the
/// Material parameter below for a multi-material mesh.
class Mesh {
public:
    /// @brief Add an element, taking ownership.
    /// @param element Heap-owned Element (any concrete subclass); ownership
    /// transfers to this Mesh, which is what lets one mesh mix element types.
    void addElement(std::unique_ptr<Element> element);

    /// @brief Assemble global residual and tangent by looping over all
    /// elements and delegating to Element::computeResidual/computeTangentStiffness.
    /// @param system Assembly target; must have been reset() beforehand.
    /// @param material Constitutive model applied to every element (see
    /// class-level note on the single-material-per-mesh limitation).
    /// @param globalDisplacement Current global displacement field, used to
    /// derive each element's local displacement slice via the DOF map.
    void assemble(GlobalSystem& system, const Material& material,
                  const Eigen::VectorXd& globalDisplacement) const;

    /// @brief Reference-configuration coordinates of every node in the mesh.
    /// @return Reference to the node coordinate array, indexed by node id.
    const std::vector<Eigen::Vector3d>& nodeCoordinates() const { return nodeCoords_; }

    /// @brief Total number of global degrees of freedom (3 per node, 3D elasticity).
    /// @return DOF count derived from the current node count.
    int numDofs() const { return static_cast<int>(dofMap_.size()) * 3; }

private:
    std::vector<Eigen::Vector3d> nodeCoords_;          ///< Reference-configuration node coordinates, indexed by node id.
    std::vector<std::unique_ptr<Element>> elements_;   ///< Owned elements; heap pointers let hex and tet elements mix in one mesh.
    std::vector<std::array<int, 3>> dofMap_;           ///< node index -> 3 global DOF indices.
};

} // namespace fem
