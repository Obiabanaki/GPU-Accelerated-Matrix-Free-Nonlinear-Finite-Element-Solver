/// @file NeoHookeanMaterial.hpp
/// @brief Concrete compressible Neo-Hookean Material implementation.
#pragma once
#include "fem/material/Material.hpp"

namespace fem {

/// @brief Compressible Neo-Hookean hyperelastic material.
///
/// Psi(F) = (mu/2)*(I1-3) - mu*ln(J) + (kappa/2)*(ln J)^2, with
/// I1 = tr(F^T*F) and J = det(F) — see NeoHookeanMaterial.cpp's file
/// comment for the derived closed-form stress/tangent and their FD
/// verification in tests/test_material.cpp.
class NeoHookeanMaterial : public Material {
public:
    /// @brief Construct with the two Neo-Hookean material parameters.
    /// @param mu Shear modulus (> 0).
    /// @param kappa Bulk modulus (> 0), controls near-incompressibility.
    NeoHookeanMaterial(double mu, double kappa);

    /// @copydoc Material::computeStress
    Eigen::Matrix3d computeStress(const Eigen::Matrix3d& F) const override;

    /// @copydoc Material::computeTangent
    Eigen::Matrix<double, 6, 6> computeTangent(const Eigen::Matrix3d& F) const override;

    /// @copydoc Material::strainEnergy
    double strainEnergy(const Eigen::Matrix3d& F) const override;

private:
    double mu_;     ///< Shear modulus.
    double kappa_;  ///< Bulk modulus (near-incompressibility control).
};

} // namespace fem
