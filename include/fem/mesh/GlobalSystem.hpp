/// @file GlobalSystem.hpp
/// @brief Assembly scratch space (GlobalSystem) — see ARCHITECTURE.md §3.
#pragma once
#include <vector>
#include <Eigen/Sparse>

namespace fem {

/// @brief Owns the global tangent stiffness matrix and residual vector for
/// one Newton iteration, plus DOF bookkeeping.
///
/// This is the assembly "scratch space" that Mesh::assemble() writes into
/// and BoundaryCondition::apply() modifies in place. Isolating it means the
/// sparse-matrix representation (triplets now, CSR-on-GPU later) can change
/// without touching Element, Material, or BoundaryCondition code.
///
/// IMPORTANT: tangent() must not be handed directly to LinearSolver::solve.
/// Wrap it in a linalg::EigenSparseOperator first — see NewtonSolver.cpp.
class GlobalSystem {
public:
    /// @brief Allocate a zeroed system for the given number of DOFs.
    /// @param numDofs Total number of global degrees of freedom.
    explicit GlobalSystem(int numDofs);

    /// @brief Zero the residual and triplet buffer before a new assembly pass.
    void reset();

    /// @brief Add a local element residual into the global residual.
    /// @param dofs Global DOF indices this element contributes to.
    /// @param localR Local residual vector, ordered to match dofs.
    void addResidual(const std::vector<int>& dofs, const Eigen::VectorXd& localR);

    /// @brief Add a local element tangent into the global triplet buffer.
    /// @param dofs Global DOF indices this element contributes to.
    /// @param localK Local tangent stiffness matrix, ordered to match dofs.
    void addTangent(const std::vector<int>& dofs, const Eigen::MatrixXd& localK);

    /// @brief Finalize triplets into a compressed sparse matrix. Call once
    /// per iteration, after all elements are assembled and before BCs apply.
    void finalize();

    /// @brief Mutable access to the assembled global tangent matrix.
    /// @return Reference to the tangent stiffness matrix.
    Eigen::SparseMatrix<double>& tangent();

    /// @brief Const access to the assembled global tangent matrix.
    /// @return Const reference to the tangent stiffness matrix.
    const Eigen::SparseMatrix<double>& tangent() const;

    /// @brief Mutable access to the global residual vector.
    /// @return Reference to the residual vector.
    Eigen::VectorXd& residual();

    /// @brief Const access to the global residual vector.
    /// @return Const reference to the residual vector.
    const Eigen::VectorXd& residual() const;

    /// @brief Total number of global degrees of freedom in this system.
    /// @return DOF count passed at construction.
    int numDofs() const { return numDofs_; }

private:
    int numDofs_;                                   ///< Total global DOF count.
    Eigen::VectorXd residual_;                       ///< Global residual vector, size numDofs_.
    std::vector<Eigen::Triplet<double>> triplets_;   ///< Buffered (row, col, value) entries, cleared in reset(), consumed in finalize().
    Eigen::SparseMatrix<double> K_;                  ///< Compressed global tangent matrix, valid after finalize().
};

} // namespace fem
