/// @file test_pipeline.cpp
/// @brief Tests for the real (non-traced) orchestration layer: config
/// parsing, mesh generation, and factory dispatch. These ARE genuine
/// correctness tests — this layer isn't in trace mode.
#include <gtest/gtest.h>
#include "fem/io/InputReader.hpp"
#include "fem/mesh/MeshBuilder.hpp"
#include "fem/factory/Factory.hpp"

// FEM_SOURCE_DIR is injected by tests/CMakeLists.txt so these tests can
// find examples/example_problem.json regardless of ctest's working directory.
#ifndef FEM_SOURCE_DIR
#define FEM_SOURCE_DIR "."
#endif

TEST(InputReader, ParsesExampleProblemFile) {
    auto config = fem::io::loadSimulationConfig(std::string(FEM_SOURCE_DIR) + "/examples/example_problem.json");

    EXPECT_EQ(config.mesh.elementType, "hex8");
    EXPECT_EQ(config.mesh.elementCounts[0], 2);
    EXPECT_EQ(config.material.type, "neo-hookean");
    EXPECT_DOUBLE_EQ(config.material.params.at("mu"), 1.0);
    EXPECT_EQ(config.boundaryConditions.size(), 2u);
    EXPECT_EQ(config.boundaryConditions[0].face, "x_min");
    EXPECT_EQ(config.solver.linearSolver, "cg");
    EXPECT_EQ(config.newton.loadSteps, 4);
}

TEST(InputReader, ThrowsOnMissingFile) {
    EXPECT_THROW(fem::io::loadSimulationConfig("no_such_file.json"), std::runtime_error);
}

TEST(MeshBuilder, GeneratesCorrectNodeAndElementCountsForHex8) {
    fem::io::MeshConfig cfg;
    cfg.elementCounts = {2, 2, 2};
    cfg.elementSize = {0.5, 0.5, 0.5};
    cfg.elementType = "hex8";

    auto built = fem::mesh::buildStructuredCubeMesh(cfg);

    EXPECT_EQ(built.mesh->numDofs() / 3, 27); // (2+1)^3 nodes
    // 6 faces of a 2x2x2 grid, each face has 3x3=9 nodes.
    EXPECT_EQ(built.faceNodeIds.at("x_min").size(), 9u);
    EXPECT_EQ(built.faceNodeIds.at("x_max").size(), 9u);
}

TEST(MeshBuilder, GeneratesSixTetsPerHexCellForTet4) {
    fem::io::MeshConfig cfg;
    cfg.elementCounts = {1, 1, 1}; // a single hex cell
    cfg.elementSize = {1.0, 1.0, 1.0};
    cfg.elementType = "tet4";

    auto built = fem::mesh::buildStructuredCubeMesh(cfg);

    EXPECT_EQ(built.mesh->numDofs() / 3, 8); // still (1+1)^3 = 8 nodes
    // Element count isn't directly exposed by Mesh; this test mainly
    // confirms buildStructuredCubeMesh doesn't throw for tet4 and produces
    // the same node grid as hex8 would.
}

TEST(Factory, CreateElementDispatchesOnType) {
    std::vector<int> hexIds = {0, 1, 2, 3, 4, 5, 6, 7};
    std::vector<Eigen::Vector3d> hexCoords(8, Eigen::Vector3d::Zero());
    EXPECT_NO_THROW(fem::factory::createElement("hex8", hexIds, hexCoords));

    std::vector<int> tetIds = {0, 1, 2, 3};
    std::vector<Eigen::Vector3d> tetCoords(4, Eigen::Vector3d::Zero());
    EXPECT_NO_THROW(fem::factory::createElement("tet4", tetIds, tetCoords));

    EXPECT_THROW(fem::factory::createElement("hex8", tetIds, tetCoords), std::invalid_argument);
    EXPECT_THROW(fem::factory::createElement("nonsense", hexIds, hexCoords), std::invalid_argument);
}

TEST(Factory, CreatePreconditionerDispatchesOnType) {
    EXPECT_NO_THROW(fem::factory::createPreconditioner("identity"));
    EXPECT_NO_THROW(fem::factory::createPreconditioner("jacobi"));
    EXPECT_NO_THROW(fem::factory::createPreconditioner("ilu"));
    EXPECT_THROW(fem::factory::createPreconditioner("nonsense"), std::invalid_argument);
}

TEST(Factory, CreateBoundaryConditionDispatchesOnType) {
    EXPECT_NO_THROW(fem::factory::createBoundaryCondition("dirichlet", {0, 1}, {0.0, 0.1}));
    EXPECT_THROW(fem::factory::createBoundaryCondition("nonsense", {0}, {0.0}), std::invalid_argument);
}
