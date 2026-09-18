/// @file MeshBuilder.cpp
/// @brief Implementation of buildStructuredCubeMesh.
///
/// Node ordering matches Hex8Element.cpp's documented VTK_HEXAHEDRON
/// convention:
///   0:(-1,-1,-1) 1:(1,-1,-1) 2:(1,1,-1) 3:(-1,1,-1)
///   4:(-1,-1, 1) 5:(1,-1, 1) 6:(1,1, 1) 7:(-1,1, 1)
///
/// For "tet4", each hex cell is split into 6 tetrahedra sharing the main
/// diagonal from local corner 0 to local corner 6 — a standard, widely
/// used hex-to-tet decomposition (see e.g. VTK's own hex->tet conversion).
#include "fem/mesh/MeshBuilder.hpp"
#include "fem/factory/Factory.hpp"
#include <iostream>
#include <stdexcept>

namespace fem::mesh {

BuiltMesh buildStructuredCubeMesh(const fem::io::MeshConfig& config) {
    if (config.type != "structured_cube") {
        throw std::invalid_argument("buildStructuredCubeMesh: unknown mesh type '" + config.type + "'");
    }

    const int nx = config.elementCounts[0];
    const int ny = config.elementCounts[1];
    const int nz = config.elementCounts[2];
    const double dx = config.elementSize[0];
    const double dy = config.elementSize[1];
    const double dz = config.elementSize[2];

    std::cout << "[MeshBuilder] generating structured_cube mesh: "
              << nx << "x" << ny << "x" << nz << " " << config.elementType
              << " elements (" << (nx + 1) * (ny + 1) * (nz + 1) << " nodes)\n";

    BuiltMesh built;
    built.mesh = std::make_unique<fem::Mesh>();

    // --- Generate the (nx+1) x (ny+1) x (nz+1) grid of nodes. ---
    auto gridIndex = [&](int I, int J, int K) {
        return I + J * (nx + 1) + K * (nx + 1) * (ny + 1);
    };

    for (int K = 0; K <= nz; ++K) {
        for (int J = 0; J <= ny; ++J) {
            for (int I = 0; I <= nx; ++I) {
                const Eigen::Vector3d coord(I * dx, J * dy, K * dz);
                const int nodeId = built.mesh->addNode(coord);
                // A node on a grid boundary belongs to every matching face
                // (edges/corners belong to more than one).
                if (I == 0)  built.faceNodeIds["x_min"].push_back(nodeId);
                if (I == nx) built.faceNodeIds["x_max"].push_back(nodeId);
                if (J == 0)  built.faceNodeIds["y_min"].push_back(nodeId);
                if (J == ny) built.faceNodeIds["y_max"].push_back(nodeId);
                if (K == 0)  built.faceNodeIds["z_min"].push_back(nodeId);
                if (K == nz) built.faceNodeIds["z_max"].push_back(nodeId);
            }
        }
    }

    // --- Generate elements, one cell at a time. ---
    // Local hex corner index -> which of the cell's 8 (I,J,K) grid corners
    // it corresponds to, matching the VTK_HEXAHEDRON convention above.
    const int cornerOffset[8][3] = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    // The 6 tets sharing the 0-6 main diagonal (local hex corner indices).
    const int tetDecomposition[6][4] = {
        {0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6},
        {0, 7, 4, 6}, {0, 4, 5, 6}, {0, 5, 1, 6},
    };

    for (int K = 0; K < nz; ++K) {
        for (int J = 0; J < ny; ++J) {
            for (int I = 0; I < nx; ++I) {
                std::array<int, 8> hexNodeIds;
                std::array<Eigen::Vector3d, 8> hexNodeCoords;
                for (int c = 0; c < 8; ++c) {
                    const int gi = gridIndex(I + cornerOffset[c][0],
                                              J + cornerOffset[c][1],
                                              K + cornerOffset[c][2]);
                    hexNodeIds[c] = gi;
                    hexNodeCoords[c] = built.mesh->nodeCoordinates()[gi];
                }

                if (config.elementType == "hex8") {
                    built.mesh->addElement(fem::factory::createElement(
                        "hex8",
                        std::vector<int>(hexNodeIds.begin(), hexNodeIds.end()),
                        std::vector<Eigen::Vector3d>(hexNodeCoords.begin(), hexNodeCoords.end())));
                } else if (config.elementType == "tet4") {
                    for (const auto& tet : tetDecomposition) {
                        std::vector<int> tetIds = {hexNodeIds[tet[0]], hexNodeIds[tet[1]],
                                                    hexNodeIds[tet[2]], hexNodeIds[tet[3]]};
                        std::vector<Eigen::Vector3d> tetCoords = {
                            hexNodeCoords[tet[0]], hexNodeCoords[tet[1]],
                            hexNodeCoords[tet[2]], hexNodeCoords[tet[3]]};
                        built.mesh->addElement(fem::factory::createElement(
                            "tet4", std::move(tetIds), std::move(tetCoords)));
                    }
                } else {
                    throw std::invalid_argument(
                        "buildStructuredCubeMesh: unknown element_type '" + config.elementType + "'");
                }
            }
        }
    }

    std::cout << "[MeshBuilder] mesh generation complete: "
              << built.mesh->numDofs() / 3 << " nodes\n";
    return built;
}

} // namespace fem::mesh
