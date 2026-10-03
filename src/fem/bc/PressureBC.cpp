/// @file PressureBC.cpp
/// @brief Consistent follower pressure load and tangent on bilinear quads.
#include "fem/bc/PressureBC.hpp"
#include <cmath>
#include <stdexcept>

namespace fem {

PressureBC::PressureBC(std::vector<PressureFacet> facets, double pressure)
    : facets_(std::move(facets)), pressure_(pressure) {
    if (facets_.empty() || !std::isfinite(pressure_) || pressure_ <= 0.0) {
        throw std::invalid_argument("PressureBC: facets must be nonempty and pressure positive");
    }
}

void PressureBC::apply(GlobalSystem& system, const Eigen::VectorXd& displacement,
                       double loadFactor) const {
    if (displacement.size() != system.numDofs()) {
        throw std::invalid_argument("PressureBC::apply: displacement size does not match system");
    }

    constexpr double g = 0.57735026918962576451;
    const double points[2] = {-g, g};
    const double pressure = pressure_ * loadFactor;
    auto& rhs = system.residual();
    auto& tangent = system.tangent();

    for (const PressureFacet& facet : facets_) {
        Eigen::Matrix<double, 3, 4> current;
        for (int node = 0; node < 4; ++node) {
            const int nodeId = facet.nodeIds[node];
            if (nodeId < 0 || 3 * nodeId + 2 >= displacement.size()) {
                throw std::out_of_range("PressureBC::apply: facet node is outside the system");
            }
            current.col(node) = facet.referenceCoordinates[node] +
                                displacement.segment<3>(3 * nodeId);
        }

        for (double eta : points) {
            for (double xi : points) {
                Eigen::Vector4d N;
                Eigen::Vector4d dN_dxi;
                Eigen::Vector4d dN_deta;
                N << 0.25 * (1.0 - xi) * (1.0 - eta),
                     0.25 * (1.0 + xi) * (1.0 - eta),
                     0.25 * (1.0 + xi) * (1.0 + eta),
                     0.25 * (1.0 - xi) * (1.0 + eta);
                dN_dxi << -0.25 * (1.0 - eta), 0.25 * (1.0 - eta),
                           0.25 * (1.0 + eta), -0.25 * (1.0 + eta);
                dN_deta << -0.25 * (1.0 - xi), -0.25 * (1.0 + xi),
                            0.25 * (1.0 + xi),  0.25 * (1.0 - xi);

                const Eigen::Vector3d tangentXi = current * dN_dxi;
                const Eigen::Vector3d tangentEta = current * dN_deta;
                const Eigen::Vector3d areaVector = tangentXi.cross(tangentEta);

                // Pressure traction is opposite the outward normal. Its derivative
                // contributes the follower-load tangent to the Newton matrix.
                for (int a = 0; a < 4; ++a) {
                    const int rowDof = 3 * facet.nodeIds[a];
                    rhs.segment<3>(rowDof) -= pressure * N[a] * areaVector;
                    for (int b = 0; b < 4; ++b) {
                        const int columnDof = 3 * facet.nodeIds[b];
                        for (int component = 0; component < 3; ++component) {
                            const Eigen::Vector3d basis = Eigen::Vector3d::Unit(component);
                            const Eigen::Vector3d areaDerivative =
                                dN_dxi[b] * basis.cross(tangentEta) +
                                dN_deta[b] * tangentXi.cross(basis);
                            for (int forceComponent = 0; forceComponent < 3; ++forceComponent) {
                                tangent.coeffRef(rowDof + forceComponent,
                                                columnDof + component) +=
                                    pressure * N[a] * areaDerivative[forceComponent];
                            }
                        }
                    }
                }
            }
        }
    }
    tangent.makeCompressed();
}

} // namespace fem