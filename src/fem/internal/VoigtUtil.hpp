/// @file VoigtUtil.hpp
/// @brief Internal helper for converting between symmetric 3x3 tensors /
/// 4th-order tensors and the project's 6-component Voigt layout.
///
/// NOT part of the public API — lives under src/, not include/, and is
/// only reachable via the PRIVATE include path fem_core grants its own
/// .cpp files (see src/CMakeLists.txt). Shared by NeoHookeanMaterial.cpp
/// and MooneyRivlinMaterial.cpp for building their computeTangent() 6x6
/// matrices.
///
/// Convention (fixed project-wide, matches Material.hpp's docs and
/// Hex8Element.cpp's file comment): Voigt index 0..5 maps to tensor pairs
/// (I,J) as [11,22,33,12,23,13]. Stress-like Voigt vectors (S, computeStress's
/// flattened output) use PLAIN tensor components. Strain-like Voigt vectors
/// use the *engineering* convention instead — shear entries (index 3..5)
/// carry a 2x factor, i.e. E_voigt = [E11,E22,E33,2*E12,2*E23,2*E13] — so
/// that S_voigt = Cvoigt * E_voigt reproduces the tensor contraction
/// S_IJ = C_IJKL*E_KL exactly with NO extra factors baked into Cvoigt
/// itself. This matches Hex8Element::strainDisplacementMatrix (its B maps
/// du to engineering-strain increments) so a Material's computeTangent()
/// output can be dropped directly into K = B^T*C*B.
#pragma once
#include <Eigen/Dense>
#include <array>
#include <utility>

namespace fem::internal {

/// @brief Voigt index (0..5) -> tensor index pair (I,J), both in {0,1,2}.
inline constexpr std::array<std::array<int, 2>, 6> kVoigtPairs = {{
    {0, 0}, {1, 1}, {2, 2}, {0, 1}, {1, 2}, {0, 2}
}};

/// @brief Flatten a symmetric 3x3 tensor to a 6-vector using kVoigtPairs
/// (plain components, no engineering doubling — for stress-like tensors).
inline Eigen::Matrix<double, 6, 1> flattenSymmetric(const Eigen::Matrix3d& T) {
    Eigen::Matrix<double, 6, 1> v;
    for (int a = 0; a < 6; ++a) v(a) = T(kVoigtPairs[a][0], kVoigtPairs[a][1]); // loop over the 6 Voigt slots
    return v;
}

/// @brief Build the 6x6 material-tangent Voigt matrix from a 4th-order
/// tensor evaluator `tensor(I, J, K, L)`, assuming both minor symmetries
/// (C_IJKL == C_JIKL == C_IJLK, true for any dS/dE derived from a strain
/// energy potential). Entries are used verbatim (no doubling) so the
/// result is meant to right-multiply an *engineering*-convention strain
/// Voigt vector (see this file's header comment) — this is what makes
/// K = B^T*C*B work in Hex8Element.cpp.
/// @param tensor Callable returning C_IJKL for indices in {0,1,2}.
template <typename Tensor4>
Eigen::Matrix<double, 6, 6> buildEngineeringVoigtTangent(Tensor4&& tensor) {
    Eigen::Matrix<double, 6, 6> Cv;
    for (int a = 0; a < 6; ++a) {         // loop over the Voigt row (I,J) pair
        for (int b = 0; b < 6; ++b) {     // loop over the Voigt column (K,L) pair
            const auto& IJ = kVoigtPairs[a];
            const auto& KL = kVoigtPairs[b];
            Cv(a, b) = tensor(IJ[0], IJ[1], KL[0], KL[1]);
        }
    }
    return Cv;
}

} // namespace fem::internal
