/// @file Material.hpp
/// @brief Abstract constitutive-model interface (Material) — see ARCHITECTURE.md §1.
#pragma once
#include <Eigen/Dense>

namespace fem {

/// @brief Abstract interface for a hyperelastic constitutive model.
///
/// A Material is a pure function of deformation: it holds material
/// parameters only, no mesh or solver state, so a single instance can be
/// shared safely across every Element that uses it.
///
/// Convention (fixed project-wide, see ARCHITECTURE.md):
///  - computeStress returns the 2nd Piola-Kirchhoff stress S (reference
///    configuration, symmetric).
///  - computeTangent returns the material tangent dS/dE, Voigt-flattened
///    to 6x6 for 3D.
class Material {
public:
    /// @brief Virtual destructor; Material is always used polymorphically.
    virtual ~Material() = default;

    /// @brief Compute the 2nd Piola-Kirchhoff stress S from deformation gradient F.
    /// @param F 3x3 deformation gradient at a Gauss point.
    /// @return S, the 2nd Piola-Kirchhoff stress tensor (3x3, symmetric).
    virtual Eigen::Matrix3d computeStress(const Eigen::Matrix3d& F) const = 0;

    /// @brief Compute the material (Lagrangian) tangent modulus C = dS/dE.
    /// @param F 3x3 deformation gradient at a Gauss point.
    /// @return 4th-order tangent, Voigt-flattened as 6x6.
    virtual Eigen::Matrix<double, 6, 6> computeTangent(const Eigen::Matrix3d& F) const = 0;

    /// @brief Strain energy density at this deformation.
    /// @param F 3x3 deformation gradient at a Gauss point.
    /// @return Scalar strain energy density Psi(F). Used for energy-based
    /// convergence checks and for numerically verifying stress = dPsi/dE
    /// in unit tests.
    virtual double strainEnergy(const Eigen::Matrix3d& F) const = 0;
};

} // namespace fem
