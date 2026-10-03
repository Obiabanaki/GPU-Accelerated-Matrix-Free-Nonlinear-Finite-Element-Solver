/// @file test_newton_solver.cpp
/// @brief Newton convergence and end-to-end analytical solution tests.
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>
#include "fem/bc/DirichletBC.hpp"
#include "fem/factory/Factory.hpp"
#include "fem/io/InputReader.hpp"
#include "fem/linalg/DirectSolver.hpp"
#include "fem/material/NeoHookeanMaterial.hpp"
#include "fem/mesh/MeshBuilder.hpp"
#include "fem/solver/NewtonSolver.hpp"

#ifndef FEM_SOURCE_DIR
#define FEM_SOURCE_DIR "."
#endif

namespace {

/// @brief Test element with an analytically solvable nonlinear equilibrium equation.
/// Its x residual is x^3 + x - 2 (root x=1) and the tangent is its exact derivative.
class CubicSpringElement final : public fem::Element {
public:
    const std::vector<int>& nodeIds() const override { return nodeIds_; }

    Eigen::VectorXd computeResidual(const Eigen::VectorXd& u,
                                   const fem::Material&) const override {
        Eigen::VectorXd residual(3);
        residual << u[0] * u[0] * u[0] + u[0] - 2.0, u[1], u[2];
        return residual;
    }

    Eigen::MatrixXd computeTangentStiffness(const Eigen::VectorXd& u,
                                             const fem::Material&) const override {
        Eigen::Matrix3d tangent = Eigen::Matrix3d::Identity();
        tangent(0, 0) = 3.0 * u[0] * u[0] + 1.0;
        return tangent;
    }

    void shapeFunctions(const Eigen::Vector3d&, Eigen::VectorXd& N,
                        Eigen::MatrixXd& dN_dxi) const override {
        N = Eigen::VectorXd::Ones(1);
        dN_dxi = Eigen::MatrixXd::Zero(1, 3);
    }

    const std::vector<fem::GaussPoint>& gaussPoints() const override {
        static const std::vector<fem::GaussPoint> points = {
            {Eigen::Vector3d::Zero(), 1.0}
        };
        return points;
    }

private:
    std::vector<int> nodeIds_{0};
};

/// @brief Two-node spring with free-node equilibrium v + v^3 = u0.
class CoupledCubicSpringElement final : public fem::Element {
public:
    const std::vector<int>& nodeIds() const override { return nodeIds_; }

    Eigen::VectorXd computeResidual(const Eigen::VectorXd& u,
                                   const fem::Material&) const override {
        Eigen::VectorXd residual = Eigen::VectorXd::Zero(6);
        residual[0] = u[0] - u[3];
        residual[3] = u[3] + u[3] * u[3] * u[3] - u[0];
        residual[4] = u[4];
        residual[5] = u[5];
        return residual;
    }

    Eigen::MatrixXd computeTangentStiffness(const Eigen::VectorXd& u,
                                             const fem::Material&) const override {
        Eigen::MatrixXd tangent = Eigen::MatrixXd::Identity(6, 6);
        tangent(0, 3) = -1.0;
        tangent(3, 0) = -1.0;
        tangent(3, 3) = 1.0 + 3.0 * u[3] * u[3];
        return tangent;
    }

    void shapeFunctions(const Eigen::Vector3d&, Eigen::VectorXd& N,
                        Eigen::MatrixXd& dN_dxi) const override {
        N = Eigen::VectorXd::Constant(2, 0.5);
        dN_dxi = Eigen::MatrixXd::Zero(2, 3);
    }

    const std::vector<fem::GaussPoint>& gaussPoints() const override {
        static const std::vector<fem::GaussPoint> points = {
            {Eigen::Vector3d::Zero(), 1.0}
        };
        return points;
    }

private:
    std::vector<int> nodeIds_{0, 1};
};

std::vector<std::unique_ptr<fem::BoundaryCondition>> makeBoundaryConditions(
    const fem::io::SimulationConfig& config,
    const fem::mesh::BuiltMesh& built) {
    std::vector<std::unique_ptr<fem::BoundaryCondition>> owners;
    for (const auto& bcConfig : config.boundaryConditions) {
        if (bcConfig.type == "pressure") {
            const auto face = built.faceQuadNodeIds.find(bcConfig.face);
            if (face == built.faceQuadNodeIds.end()) {
                throw std::runtime_error("pressure test references unknown mesh face");
            }
            std::vector<fem::PressureFacet> facets;
            for (const auto& nodeIds : face->second) {
                fem::PressureFacet facet;
                facet.nodeIds = nodeIds;
                for (int node = 0; node < 4; ++node) {
                    facet.referenceCoordinates[node] = built.mesh->nodeCoordinates()[nodeIds[node]];
                }
                facets.push_back(std::move(facet));
            }
            owners.push_back(fem::factory::createPressureBoundaryCondition(
                std::move(facets), bcConfig.pressure));
            continue;
        }
        const auto face = built.faceNodeIds.find(bcConfig.face);
        if (face == built.faceNodeIds.end()) throw std::runtime_error("test references unknown mesh face");
        std::vector<int> dofs;
        std::vector<double> values;
        for (int nodeId : face->second) {
            for (int component = 0; component < 3; ++component) {
                if (bcConfig.components[component]) {
                    dofs.push_back(3 * nodeId + component);
                    values.push_back(bcConfig.value[component]);
                }
            }
        }
        owners.push_back(fem::factory::createBoundaryCondition(
            bcConfig.type, std::move(dofs), std::move(values)));
    }
    return owners;
}

struct RadialState {
    double radius;
    double nominalRadialStress;
};

double solveRadialStretch(double referenceRadius, const RadialState& state,
                          double shearModulus, double bulkModulus) {
    const double hoopStretch = state.radius / referenceRadius;
    auto residualAt = [&](double radialStretch) {
        const double J = radialStretch * hoopStretch;
        const double stressRR = shearModulus * (1.0 - 1.0 / (radialStretch * radialStretch))
            + bulkModulus * std::log(J) / (radialStretch * radialStretch);
        return radialStretch * stressRR - state.nominalRadialStress;
    };
    double lowerStretch = 1e-6;
    double upperStretch = 1.0;
    double lowerResidual = residualAt(lowerStretch);
    double upperResidual = residualAt(upperStretch);
    while (upperResidual < 0.0 && upperStretch < 1e4) {
        upperStretch *= 2.0;
        upperResidual = residualAt(upperStretch);
    }
    if (lowerResidual * upperResidual > 0.0) {
        throw std::runtime_error("could not bracket positive radial stretch");
    }

    double radialStretch = std::clamp(1.0, lowerStretch, upperStretch);
    for (int iteration = 0; iteration < 30; ++iteration) {
        const double J = radialStretch * hoopStretch;
        const double residual = residualAt(radialStretch);
        if (std::abs(residual) < 1e-9 * (1.0 + std::abs(state.nominalRadialStress))) {
            return radialStretch;
        }
        const double derivative = shearModulus *
                (1.0 + 1.0 / (radialStretch * radialStretch))
            + bulkModulus * (1.0 - std::log(J)) /
                (radialStretch * radialStretch);
        if (residual > 0.0) upperStretch = radialStretch;
        else lowerStretch = radialStretch;
        const double newtonStretch = radialStretch - residual / derivative;
        radialStretch = (newtonStretch >= lowerStretch && newtonStretch <= upperStretch)
            ? newtonStretch : 0.5 * (lowerStretch + upperStretch);
    }
    throw std::runtime_error("radial reference stretch solve did not converge");
}

std::vector<double> analyticalCylinderRadii(double innerRadius, double outerRadius,
                                            double pressure, int radialElements,
                                            double shearModulus, double bulkModulus) {
    const int stepsPerElement = 24;
    const int steps = radialElements * stepsPerElement;
    const double stepSize = (outerRadius - innerRadius) / steps;

    auto derivative = [&](double referenceRadius, const RadialState& state) {
        const double radialStretch = solveRadialStretch(
            referenceRadius, state, shearModulus, bulkModulus);
        const double hoopStretch = state.radius / referenceRadius;
        const double J = radialStretch * hoopStretch;
        const double stressTheta = shearModulus * (1.0 - 1.0 / (hoopStretch * hoopStretch))
            + bulkModulus * std::log(J) / (hoopStretch * hoopStretch);
        const double nominalHoopStress = hoopStretch * stressTheta;
        return RadialState{radialStretch,
                           (nominalHoopStress - state.nominalRadialStress) / referenceRadius};
    };

    auto integrate = [&](double deformedInnerRadius, std::vector<double>* sampledRadii) {
        const double innerHoopStretch = deformedInnerRadius / innerRadius;
        RadialState state{deformedInnerRadius, -pressure * innerHoopStretch};
        solveRadialStretch(innerRadius, state, shearModulus, bulkModulus);
        if (sampledRadii != nullptr) {
            sampledRadii->assign(static_cast<std::size_t>(radialElements + 1), 0.0);
            (*sampledRadii)[0] = state.radius;
        }

        for (int step = 0; step < steps; ++step) {
            const double R = innerRadius + step * stepSize;
            const RadialState k1 = derivative(R, state);
            const RadialState k2 = derivative(R + 0.5 * stepSize,
                {state.radius + 0.5 * stepSize * k1.radius,
                 state.nominalRadialStress + 0.5 * stepSize * k1.nominalRadialStress});
            const RadialState k3 = derivative(R + 0.5 * stepSize,
                {state.radius + 0.5 * stepSize * k2.radius,
                 state.nominalRadialStress + 0.5 * stepSize * k2.nominalRadialStress});
            const RadialState k4 = derivative(R + stepSize,
                {state.radius + stepSize * k3.radius,
                 state.nominalRadialStress + stepSize * k3.nominalRadialStress});
            state.radius += stepSize / 6.0 *
                (k1.radius + 2.0 * k2.radius + 2.0 * k3.radius + k4.radius);
            state.nominalRadialStress += stepSize / 6.0 *
                (k1.nominalRadialStress + 2.0 * k2.nominalRadialStress +
                 2.0 * k3.nominalRadialStress + k4.nominalRadialStress);
            if (sampledRadii != nullptr && (step + 1) % stepsPerElement == 0) {
                (*sampledRadii)[static_cast<std::size_t>((step + 1) / stepsPerElement)] = state.radius;
            }
        }
        return state.nominalRadialStress;
    };

    double lowerRadius = innerRadius;
    double upperRadius = innerRadius + (outerRadius - innerRadius);
    double lowerResidual = integrate(lowerRadius, nullptr);
    double upperResidual = integrate(upperRadius, nullptr);
    for (int expansion = 0; lowerResidual * upperResidual > 0.0 && expansion < 8; ++expansion) {
        upperRadius += outerRadius - innerRadius;
        upperResidual = integrate(upperRadius, nullptr);
    }
    if (lowerResidual * upperResidual > 0.0) {
        throw std::runtime_error("could not bracket analytical cylinder inflation solution");
    }
    for (int iteration = 0; iteration < 36; ++iteration) {
        const double middleRadius = 0.5 * (lowerRadius + upperRadius);
        const double middleResidual = integrate(middleRadius, nullptr);
        if (lowerResidual * middleResidual <= 0.0) {
            upperRadius = middleRadius;
            upperResidual = middleResidual;
        } else {
            lowerRadius = middleRadius;
            lowerResidual = middleResidual;
        }
    }
    std::vector<double> radii;
    integrate(0.5 * (lowerRadius + upperRadius), &radii);
    return radii;
}

} // namespace

TEST(NewtonSolver, SolvesNonlinearEquationAtItsAnalyticalRoot) {
    fem::Mesh mesh;
    mesh.addNode(Eigen::Vector3d::Zero());
    mesh.addElement(std::make_unique<CubicSpringElement>());
    fem::NeoHookeanMaterial material(1.0, 10.0);
    fem::linalg::DirectSolver linearSolver;
    fem::NewtonSolver solver(mesh, material, linearSolver, {});

    const Eigen::VectorXd displacement = solver.solve(1, 1e-12, 1e-12, 20);

    ASSERT_EQ(displacement.size(), 3);
    EXPECT_NEAR(displacement[0], 1.0, 1e-10);
    EXPECT_NEAR(displacement[1], 0.0, 1e-12);
    EXPECT_NEAR(displacement[2], 0.0, 1e-12);
    ASSERT_EQ(solver.convergenceHistory().size(), 1u);
    EXPECT_LT(solver.convergenceHistory()[0].back(), 1e-12);
}

TEST(NewtonSolver, CutsBackLargeLoadIncrementAndRecovers) {
    fem::Mesh mesh;
    mesh.addNode(Eigen::Vector3d::Zero());
    mesh.addNode(Eigen::Vector3d::UnitX());
    mesh.addElement(std::make_unique<CoupledCubicSpringElement>());
    fem::NeoHookeanMaterial material(1.0, 10.0);
    fem::linalg::DirectSolver linearSolver;
    fem::DirichletBC prescribedInput({0}, {2.0});
    std::vector<std::reference_wrapper<fem::BoundaryCondition>> boundaryConditions = {
        prescribedInput
    };
    fem::NewtonSolver solver(mesh, material, linearSolver, boundaryConditions);

    const Eigen::VectorXd displacement = solver.solve(1, 1e-8, 1e-8, 6);

    EXPECT_NEAR(displacement[0], 2.0, 1e-8);
    EXPECT_NEAR(displacement[3], 1.0, 1e-7);
    ASSERT_EQ(solver.convergenceHistory().size(), 1u);
    EXPECT_GT(solver.convergenceHistory()[0].size(), 6u);
    EXPECT_LT(solver.convergenceHistory()[0].back(), 1e-8);
}

TEST(NewtonSolver, ReproducesAnalyticalAffineHex8Displacement) {
    const auto config = fem::io::loadSimulationConfig(
        std::string(FEM_SOURCE_DIR) + "/examples/analytical_newton.json");
    auto built = fem::mesh::buildStructuredCubeMesh(config.mesh);
    auto material = fem::factory::createMaterial(config.material.type, config.material.params);
    auto preconditioner = fem::factory::createPreconditioner(config.solver.preconditioner);
    auto linearSolver = fem::factory::createLinearSolver(config.solver.linearSolver, *preconditioner);
    auto bcOwners = makeBoundaryConditions(config, built);
    std::vector<std::reference_wrapper<fem::BoundaryCondition>> bcRefs;
    for (auto& bc : bcOwners) bcRefs.push_back(*bc);

    fem::NewtonSolver solver(*built.mesh, *material, *linearSolver, bcRefs);
    const Eigen::VectorXd displacement = solver.solve(
        config.newton.loadSteps, config.newton.residualTolerance,
        config.newton.displacementTolerance, config.newton.maxIterationsPerStep);

    const auto& coordinates = built.mesh->nodeCoordinates();
    ASSERT_EQ(displacement.size(), static_cast<Eigen::Index>(coordinates.size() * 3));
    for (std::size_t node = 0; node < coordinates.size(); ++node) {
        const double expectedX = 0.1 * coordinates[node].x();
        EXPECT_NEAR(displacement[static_cast<Eigen::Index>(3 * node)], expectedX, 1e-10);
        EXPECT_NEAR(displacement[static_cast<Eigen::Index>(3 * node + 1)], 0.0, 1e-10);
        EXPECT_NEAR(displacement[static_cast<Eigen::Index>(3 * node + 2)], 0.0, 1e-10);
    }
}

TEST(NewtonSolver, CylinderInflationMatchesRadialEquilibriumReference) {
    const auto config = fem::io::loadSimulationConfig(
        std::string(FEM_SOURCE_DIR) + "/examples/cylinder_inflation.json");
    auto built = fem::mesh::buildMesh(config.mesh);
    auto materialBase = fem::factory::createMaterial(config.material.type, config.material.params);
    auto* material = dynamic_cast<fem::NeoHookeanMaterial*>(materialBase.get());
    ASSERT_NE(material, nullptr);
    auto preconditioner = fem::factory::createPreconditioner(config.solver.preconditioner);
    auto linearSolver = fem::factory::createLinearSolver(config.solver.linearSolver, *preconditioner);
    auto bcOwners = makeBoundaryConditions(config, built);
    std::vector<std::reference_wrapper<fem::BoundaryCondition>> bcRefs;
    for (auto& bc : bcOwners) bcRefs.push_back(*bc);

    fem::NewtonSolver solver(*built.mesh, *material, *linearSolver, bcRefs);
    const Eigen::VectorXd displacement = solver.solve(
        config.newton.loadSteps, config.newton.residualTolerance,
        config.newton.displacementTolerance, config.newton.maxIterationsPerStep);

    const int radialElements = config.mesh.elementCounts[0];
    const int angularElements = config.mesh.elementCounts[1];
    const int radialNodes = radialElements + 1;
    const int middleAngularIndex = angularElements / 2;
    const auto analyticalRadii = analyticalCylinderRadii(
        config.mesh.innerRadius, config.mesh.outerRadius,
        config.boundaryConditions.back().pressure, radialElements,
        config.material.params.at("mu"), config.material.params.at("kappa"));

    double maximumRadialError = 0.0;
    for (int radial = 0; radial <= radialElements; ++radial) {
        const int node = radial + middleAngularIndex * radialNodes;
        const Eigen::Vector3d X = built.mesh->nodeCoordinates()[node];
        const double theta = std::atan2(X.y(), X.x());
        const Eigen::Vector3d radialDirection(std::cos(theta), std::sin(theta), 0.0);
        const double numericalRadialDisplacement =
            radialDirection.dot(displacement.segment<3>(3 * node));
        const double referenceRadius = X.head<2>().norm();
        const double analyticalDisplacement = analyticalRadii[radial] - referenceRadius;
        if (radial == 0 || radial == radialElements / 2 || radial == radialElements) {
            std::cout << "Cylinder R=" << referenceRadius
                      << ": FE u_r=" << numericalRadialDisplacement
                      << ", radial-reference u_r=" << analyticalDisplacement << " m\n";
        }
        maximumRadialError = std::max(maximumRadialError,
            std::abs(numericalRadialDisplacement - analyticalDisplacement));
        EXPECT_NEAR(numericalRadialDisplacement, analyticalDisplacement, 0.003)
            << "radial layer " << radial << ", R=" << referenceRadius;
    }
        std::cout << "Cylinder radial reference maximum displacement error: "
                            << maximumRadialError << " m\n";
    EXPECT_LT(maximumRadialError, 0.003);
}

TEST(NewtonSolver, RejectsInvalidControls) {
    fem::Mesh mesh;
    mesh.addNode(Eigen::Vector3d::Zero());
    mesh.addElement(std::make_unique<CubicSpringElement>());
    fem::NeoHookeanMaterial material(1.0, 10.0);
    fem::linalg::DirectSolver linearSolver;
    fem::NewtonSolver solver(mesh, material, linearSolver, {});

    EXPECT_THROW(solver.solve(0, 1e-6, 1e-6, 10), std::invalid_argument);
    EXPECT_THROW(solver.solve(1, 1e-6, 1e-6, 0), std::invalid_argument);
    EXPECT_THROW(solver.solve(1, 0.0, 1e-6, 10), std::invalid_argument);
}