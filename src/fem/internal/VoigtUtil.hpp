/// @file VoigtUtil.hpp
/// @brief Internal helper for converting between symmetric 3x3 tensor
/// components and the project's 6-component Voigt ordering.
///
/// NOT part of the public API — lives under src/, not include/, and is
/// only reachable via the PRIVATE include path fem_core grants its own
/// .cpp files (see src/CMakeLists.txt). Shared by NeoHookeanMaterial.cpp
/// (building the Voigt tangent matrix) and Hex8Element.cpp (contracting
/// it back into a 4th-order tensor for the material stiffness).
///
/// Convention (fixed project-wide, matches Material.hpp's docs): Voigt
/// index 0..5 maps to tensor pairs (I,J) as [11,22,33,12,13,23]. Both
/// stress/strain Voigt vectors and the Voigt tangent matrix use PLAIN
/// tensor components — no "engineering strain" factor-of-2 doubling on
/// the shear entries. Element code must never assume the doubled
/// convention; see Hex8Element.cpp's tangentTensor() for how the plain
/// convention is consumed correctly via the full 4-index contraction.
#pragma once
#include <Eigen/Dense>
#include <array>

namespace fem::internal {

/// @brief Voigt index (0..5) -> tensor index pair (I,J), both in {0,1,2}.
inline constexpr std::array<std::array<int, 2>, 6> kVoigtPairs = {{
    {0, 0}, {1, 1}, {2, 2}, {0, 1}, {0, 2}, {1, 2}
}};

/// @brief Tensor index pair (I,J), both in {0,1,2} -> Voigt index (0..5).
inline int voigtIndex(int I, int J) {
    if (I == J) return I;
    if ((I == 0 && J == 1) || (I == 1 && J == 0)) return 3;
    if ((I == 0 && J == 2) || (I == 2 && J == 0)) return 4;
    return 5; // (1,2) or (2,1)
}

/// @brief Flatten a symmetric 3x3 tensor to a 6-vector using kVoigtPairs.
inline Eigen::Matrix<double, 6, 1> flattenSymmetric(const Eigen::Matrix3d& T) {
    Eigen::Matrix<double, 6, 1> v;
    for (int a = 0; a < 6; ++a) {
        v(a) = T(kVoigtPairs[a][0], kVoigtPairs[a][1]);
    }
    return v;
}

/// @brief Expand a 6-vector (plain tensor convention) back to a symmetric 3x3 tensor.
inline Eigen::Matrix3d expandSymmetric(const Eigen::Matrix<double, 6, 1>& v) {
    Eigen::Matrix3d T;
    for (int a = 0; a < 6; ++a) {
        int I = kVoigtPairs[a][0], J = kVoigtPairs[a][1];
        T(I, J) = v(a);
        T(J, I) = v(a);
    }
    return T;
}

/// @brief Look up C_{IJKL} from a 6x6 Voigt tangent matrix built with the
/// plain-tensor-component convention above (no doubling anywhere).
inline double tangentTensorEntry(const Eigen::Matrix<double, 6, 6>& Cv,
                                  int I, int J, int K, int L) {
    return Cv(voigtIndex(I, J), voigtIndex(K, L));
}

} // namespace fem::internal
