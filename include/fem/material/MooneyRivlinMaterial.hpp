/// @file MooneyRivlinMaterial.hpp
/// @brief Concrete two-parameter Mooney-Rivlin Material implementation.
#pragma once
#include "fem/material/Material.hpp"

namespace fem {

/// @brief Two-parameter Mooney-Rivlin hyperelastic material.
///
/// Psi(F) = c10*(I1-3) + c01*(I2-3) + (kappa/2)*(ln J)^2, with I1/I2 the
/// first/second invariants of C = F^T*F and J = det(F) — see
/// MooneyRivlinMaterial.cpp's file comment for the derived closed-form
/// stress/tangent (and a documented limitation of this raw-invariant
/// formulation) plus their FD verification in tests/test_material.cpp.
class MooneyRivlinMaterial : public Material {
public:
    /// @brief Construct with the three Mooney-Rivlin material parameters.
    /// @param c10 First Mooney-Rivlin parameter.
    /// @param c01 Second Mooney-Rivlin parameter.
    /// @param kappa Bulk modulus, controls near-incompressibility.
    MooneyRivlinMaterial(double c10, double c01, double kappa);

    /// @copydoc Material::computeStress
    Eigen::Matrix3d computeStress(const Eigen::Matrix3d& F) const override;

    /// @copydoc Material::computeTangent
    Eigen::Matrix<double, 6, 6> computeTangent(const Eigen::Matrix3d& F) const override;

    /// @copydoc Material::strainEnergy
    double strainEnergy(const Eigen::Matrix3d& F) const override;

private:
    double c10_;    ///< First Mooney-Rivlin parameter.
    double c01_;    ///< Second Mooney-Rivlin parameter.
    double kappa_;  ///< Bulk modulus (near-incompressibility control).
};

} // namespace fem
