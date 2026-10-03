/// @file MeshBuilder.hpp
/// @brief Generates a structured cube or cylindrical-sector Mesh from MeshConfig.
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

/// @brief Generated Mesh with boundary nodes and oriented surface facets.
struct BuiltMesh {
    std::unique_ptr<fem::Mesh> mesh;
    std::map<std::string, std::vector<int>> faceNodeIds; ///< "x_min"/"x_max"/"y_min"/"y_max"/"z_min"/"z_max" -> node ids.
    std::map<std::string, std::vector<std::array<int, 4>>> faceQuadNodeIds; ///< Oriented boundary quads for surface loads.
};

/// @brief Generate a structured cube mesh per the given configuration.
/// @param config Mesh generation parameters (element counts, size, type).
/// @return The generated mesh and its face node-id sets.
/// @throws std::invalid_argument if config.type or config.elementType isn't recognized.
BuiltMesh buildStructuredCubeMesh(const fem::io::MeshConfig& config);

/// @brief Generate a structured 90-degree cylindrical annulus sector of Hex8 elements.
/// @param config Counts are radial/angular/axial; radii and axialLength define geometry.
/// @return Mesh, face-node sets, and oriented inner/outer surface quadrilaterals.
/// @throws std::invalid_argument if geometry or element type is unsupported.
BuiltMesh buildStructuredCylinderMesh(const fem::io::MeshConfig& config);

/// @brief Dispatch to the structured mesh builder named by config.type.
BuiltMesh buildMesh(const fem::io::MeshConfig& config);

} // namespace fem::mesh
