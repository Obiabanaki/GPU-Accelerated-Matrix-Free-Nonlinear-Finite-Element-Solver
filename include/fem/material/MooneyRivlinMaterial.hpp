/// @file MooneyRivlinMaterial.hpp
/// @brief Concrete two-parameter Mooney-Rivlin Material implementation.
#pragma once
#include "fem/material/Material.hpp"

namespace fem {

/// @brief Two-parameter Mooney-Rivlin hyperelastic material.
///
/// Step 1.5: second Material subclass, used to demonstrate near-
/// incompressibility handling (selective reduced integration / mixed u-p)
/// and to prove the Material abstraction isn't a single-implementation
/// interface.
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
