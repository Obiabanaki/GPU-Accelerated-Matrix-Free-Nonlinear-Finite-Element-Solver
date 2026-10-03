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
#include <cmath>
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

BuiltMesh buildStructuredCylinderMesh(const fem::io::MeshConfig& config) {
    if (config.type != "structured_cylinder" || config.elementType != "hex8" ||
        config.elementCounts[0] <= 0 || config.elementCounts[1] <= 0 ||
        config.elementCounts[2] <= 0 || config.innerRadius <= 0.0 ||
        config.outerRadius <= config.innerRadius || config.axialLength <= 0.0) {
        throw std::invalid_argument("buildStructuredCylinderMesh: invalid cylinder configuration");
    }

    const int radialCount = config.elementCounts[0];
    const int angularCount = config.elementCounts[1];
    const int axialCount = config.elementCounts[2];
    const int radialNodes = radialCount + 1;
    const int angularNodes = angularCount + 1;
    constexpr double quarterTurn = 1.57079632679489661923;
    auto nodeIndex = [=](int radial, int angular, int axial) {
        return radial + angular * radialNodes + axial * radialNodes * angularNodes;
    };

    BuiltMesh built;
    built.mesh = std::make_unique<fem::Mesh>();
    for (int axial = 0; axial <= axialCount; ++axial) {
        const double z = config.axialLength * axial / axialCount;
        for (int angular = 0; angular <= angularCount; ++angular) {
            const double theta = quarterTurn * angular / angularCount;
            for (int radial = 0; radial <= radialCount; ++radial) {
                const double radius = config.innerRadius +
                    (config.outerRadius - config.innerRadius) * radial / radialCount;
                const int node = built.mesh->addNode(
                    Eigen::Vector3d(radius * std::cos(theta), radius * std::sin(theta), z));
                if (radial == 0) built.faceNodeIds["r_min"].push_back(node);
                if (radial == radialCount) built.faceNodeIds["r_max"].push_back(node);
                if (angular == 0) built.faceNodeIds["theta_min"].push_back(node);
                if (angular == angularCount) built.faceNodeIds["theta_max"].push_back(node);
                if (axial == 0) built.faceNodeIds["z_min"].push_back(node);
                if (axial == axialCount) built.faceNodeIds["z_max"].push_back(node);
            }
        }
    }

    // Build Hex8 cells in radial/angular/axial order and retain cavity facets.
    for (int axial = 0; axial < axialCount; ++axial) {
        for (int angular = 0; angular < angularCount; ++angular) {
            for (int radial = 0; radial < radialCount; ++radial) {
                const std::array<int, 8> ids = {
                    nodeIndex(radial, angular, axial),
                    nodeIndex(radial + 1, angular, axial),
                    nodeIndex(radial + 1, angular + 1, axial),
                    nodeIndex(radial, angular + 1, axial),
                    nodeIndex(radial, angular, axial + 1),
                    nodeIndex(radial + 1, angular, axial + 1),
                    nodeIndex(radial + 1, angular + 1, axial + 1),
                    nodeIndex(radial, angular + 1, axial + 1),
                };
                std::array<Eigen::Vector3d, 8> coordinates;
                for (int corner = 0; corner < 8; ++corner) {
                    coordinates[corner] = built.mesh->nodeCoordinates()[ids[corner]];
                }
                built.mesh->addElement(fem::factory::createElement(
                    "hex8", std::vector<int>(ids.begin(), ids.end()),
                    std::vector<Eigen::Vector3d>(coordinates.begin(), coordinates.end())));

                if (radial == 0) {
                    // This order gives the cavity wall's outward normal into the hole.
                    built.faceQuadNodeIds["r_min"].push_back(
                        {ids[0], ids[4], ids[7], ids[3]});
                }
                if (radial == radialCount - 1) {
                    built.faceQuadNodeIds["r_max"].push_back(
                        {ids[1], ids[2], ids[6], ids[5]});
                }
            }
        }
    }
    return built;
}

BuiltMesh buildMesh(const fem::io::MeshConfig& config) {
    if (config.type == "structured_cube") return buildStructuredCubeMesh(config);
    if (config.type == "structured_cylinder") return buildStructuredCylinderMesh(config);
    throw std::invalid_argument("buildMesh: unknown mesh type '" + config.type + "'");
}

} // namespace fem::mesh
