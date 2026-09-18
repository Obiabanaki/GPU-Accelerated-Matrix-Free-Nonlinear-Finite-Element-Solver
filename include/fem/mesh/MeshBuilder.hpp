/// @file MeshBuilder.hpp
/// @brief Generates a structured-grid Mesh from a MeshConfig.
///
/// This is real, non-traced code: mesh topology generation (node
/// coordinates, element connectivity) is bookkeeping, not FEM physics, so
/// it's implemented for real even while the rest of the pipeline is in
/// trace mode — this is what lets the input file's mesh options actually
/// produce a different mesh, not just a different log line.
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "fem/mesh/Mesh.hpp"
#include "fem/io/SimulationConfig.hpp"

namespace fem::mesh {

/// @brief A generated Mesh, plus the node ids on each of its 6 outer faces
/// — needed to resolve a BoundaryConditionConfig's "face" string into
/// actual DOF indices.
struct BuiltMesh {
    std::unique_ptr<fem::Mesh> mesh;
    std::map<std::string, std::vector<int>> faceNodeIds; ///< "x_min"/"x_max"/"y_min"/"y_max"/"z_min"/"z_max" -> node ids.
};

/// @brief Generate a structured cube mesh per the given configuration.
/// @param config Mesh generation parameters (element counts, size, type).
/// @return The generated mesh and its face node-id sets.
/// @throws std::invalid_argument if config.type or config.elementType isn't recognized.
BuiltMesh buildStructuredCubeMesh(const fem::io::MeshConfig& config);

} // namespace fem::mesh
