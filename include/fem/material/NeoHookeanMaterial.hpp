/// @file NeoHookeanMaterial.hpp
/// @brief Concrete compressible Neo-Hookean Material implementation.
#pragma once
#include "fem/material/Material.hpp"

namespace fem {

/// @brief Compressible Neo-Hookean hyperelastic material.
///
/// Step 0-pre / Phase 0.5: the first, FD-verified Material implementation.
/// Parameterized by shear modulus mu and bulk modulus kappa (or lambda,
/// depending on the compressible formulation chosen — fix and document
/// which one before implementing computeStress).
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
