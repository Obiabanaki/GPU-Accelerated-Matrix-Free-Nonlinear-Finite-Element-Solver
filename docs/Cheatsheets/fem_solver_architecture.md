# Nonlinear FEM Solver — Detailed Class Architecture

This document expands the architecture sketch in `nonlinear_FEM_solver_project_plan.md` into
full class specifications: properties, method signatures, and — for each one — *why it exists*
and *why its boundaries are drawn where they are*. Read this before writing Phase 1 code; the
goal is that nothing in Phase 1–3 ever needs to reach across a boundary drawn here.

A running theme: **every class either *is* a mathematical object (Material, Element) or
*orchestrates* mathematical objects (Mesh, NewtonSolver) — never both.** Whenever a class starts
doing both, that's a signal the abstraction has leaked.

> **Revision note:** this version fixes three places where the first draft quietly reverted to
> v1 patterns that v2 explicitly rejects: (1) `LinearSolver` took a raw matrix instead of a
> `LinearOperator`, breaking the matrix-free/GPU story; (2) `ConjugateGradientSolver` depended on
> `ComputeBackend` directly, coupling it to the backend instead of hiding the backend behind the
> operator; (3) `NewtonSolver` held boundary conditions as raw pointers instead of
> `reference_wrapper`. See §6a (`LinearOperator`) and the ownership note under §11
> (`NewtonSolver`) for the fixes, and §12 for a real open question v2's own diagram doesn't
> resolve (preconditioning + matrix-free).

### How to read this document during Phase 0-pre vs. Phase 0.5

**This document describes the Phase 0.5 end-state — do not build against it during Phase 0-pre.**
v2's entire restructuring rationale is that you cannot design `Material`/`Element` correctly
before `NeoHookeanMaterial` and `Hex8Element` exist as concrete code and you've felt, firsthand,
what `Element` actually needs from `Material`. If you start Phase 0-pre by writing
`NeoHookeanMaterial : public Material` against the interface below, you've reintroduced the
premature-abstraction mistake v2 exists to avoid — just with this document standing in for the
upfront design v1 got wrong.

Correct sequencing:
1. **Phase 0-pre:** write `NeoHookeanMaterial` and `Hex8Element` as bare concrete classes, no
   base class, no virtual methods. Get the FD-verified tangent and patch test passing.
2. **Phase 0.5:** *then* open this document, retrofit `NeoHookeanMaterial : public Material` and
   `Hex8Element : public Element`, and confirm the Phase 0-pre tests still pass unmodified. That
   unmodified-pass is the actual proof the interface is sound — not agreement with this document.
3. Everything from `LinearOperator` onward (§6a–§9) is genuinely new code with no concrete
   precedent from Phase 0-pre, since Phase 0-pre never touches a linear solver abstraction at
   all (it calls `Eigen::.solve()` directly, per the plan). Build those directly against the
   interfaces below — there's no "concrete-first" step to do for them, because Phase 0-pre's
   dense Newton solve doesn't exercise that code path.

---

## 1. `Material` hierarchy

### Why this class exists
A material model is a pure function of state: given a deformation, return stress and its
derivative. It has no business knowing about elements, meshes, or solvers. Isolating it means
Step 1.1 can be unit-tested against a closed-form solution with zero FEM machinery in the way —
and it means a rubber engineer's domain expertise (Neo-Hookean, Mooney-Rivlin, Ogden...) plugs
into the code as pure constitutive math, not tangled into element loops.

```cpp
/// @brief Abstract interface for a hyperelastic constitutive model.
///
/// A Material knows only about deformation and stress — nothing about mesh
/// topology or solver state. This keeps constitutive law swaps (Neo-Hookean
/// -> Mooney-Rivlin -> Ogden) confined to a single new subclass.
class Material {
public:
    virtual ~Material() = default;

    /// @brief Compute the 2nd Piola-Kirchhoff stress S from deformation gradient F.
    /// @param F 3x3 deformation gradient at a Gauss point.
    /// @return S, the 2nd Piola-Kirchhoff stress tensor (3x3, symmetric).
    virtual Eigen::Matrix3d computeStress(const Eigen::Matrix3d& F) const = 0;

    /// @brief Compute the material (Lagrangian) tangent modulus C = dS/dE.
    /// @param F 3x3 deformation gradient at a Gauss point.
    /// @return 4th-order tangent, stored Voigt-flattened as 6x6.
    virtual Eigen::Matrix<double,6,6> computeTangent(const Eigen::Matrix3d& F) const = 0;

    /// @brief Strain energy density at this deformation (needed for energy-based
    /// convergence checks and for verifying stress = dPsi/dE numerically in tests).
    virtual double strainEnergy(const Eigen::Matrix3d& F) const = 0;
};
```

**Properties (on concrete subclasses, not the base):** material parameters only —
e.g. `NeoHookeanMaterial` holds `mu_` (shear modulus) and `lambda_` (Lamé's first parameter,
or `kappa_` bulk modulus depending on your compressible formulation). No mutable state,
no mesh references. This makes `Material` instances trivially shareable (one instance,
many elements) and thread-safe by construction — relevant later when you parallelize assembly.

**Subclasses:**
- `NeoHookeanMaterial` — Step 1.1.
- `MooneyRivlinMaterial` — Step 1.5; exists specifically to *prove* the base class isn't a
  single-implementation abstraction. If you only ever write one Material subclass, an
  interviewer's natural next question is "then why the abstract base?" — Mooney-Rivlin answers it.

**Design check:** `computeStress` and `computeTangent` are `const` — a material model with
mutable internal state (other than a cache) is a design smell. If you later add a
history-dependent material (viscoelastic, plastic), that state belongs in a separate
`MaterialState` object passed by reference, not in the `Material` object itself — otherwise a
single `NeoHookeanMaterial` instance can no longer be shared safely across elements holding
different histories.

> **Likely interview question:** "Why separate `computeStress` and `computeTangent` into two
> calls instead of one that returns both?" — Answer: some solvers (e.g. modified Newton, or a
> quasi-Newton/BFGS approach) never need the tangent at every iteration, so forcing its
> computation every call wastes work. Splitting them lets a caller skip the tangent when it isn't
> needed. The tradeoff is redundant computation of shared intermediate quantities (like invariants
> of F) if both are called back-to-back — worth mentioning you're aware of it, and that a
> `computeStressAndTangent` combined call, or an internal per-call cache, is the fix if profiling
> shows it matters.

---

## 2. `Element` hierarchy

### Why this class exists
An element owns everything about *how* a material's stress becomes a nodal force vector and
stiffness contribution for one specific geometry (hex, tet, ...): shape functions, quadrature
rule, B-matrix (strain-displacement operator), and the integral loop. It takes a `Material&`
rather than owning one, because the same `Hex8Element` type must work with *any* material — the
element defines geometry/kinematics, the material defines constitutive response, and those are
orthogonal concerns.

```cpp
/// @brief Abstract interface for one finite element's local kinematics and integration.
///
/// An Element converts nodal displacements + a Material into a local residual
/// and tangent stiffness. It never owns a Material — one is passed in per call
/// — so a single Hex8Element type works unmodified with any Material subclass.
class Element {
public:
    virtual ~Element() = default;

    /// @brief Global node indices this element connects (defines DOF mapping).
    virtual const std::vector<int>& nodeIds() const = 0;

    /// @brief Local internal-force residual vector for the current nodal displacement u.
    /// @param u Nodal displacements for this element's nodes, local ordering.
    /// @param material Constitutive model evaluated at each Gauss point.
    /// @return Local residual vector, size = numNodes * numDofPerNode.
    virtual Eigen::VectorXd computeResidual(const Eigen::VectorXd& u,
                                             const Material& material) const = 0;

    /// @brief Local tangent stiffness matrix (material + geometric stiffness).
    virtual Eigen::MatrixXd computeTangentStiffness(const Eigen::VectorXd& u,
                                                      const Material& material) const = 0;

    /// @brief Shape function values and parametric-space gradients at a point.
    virtual void shapeFunctions(const Eigen::Vector3d& xi,
                                 Eigen::VectorXd& N,
                                 Eigen::MatrixXd& dN_dxi) const = 0;

    /// @brief Gauss quadrature points and weights for this element's integration rule.
    virtual const std::vector<GaussPoint>& gaussPoints() const = 0;
};
```

**Properties:** reference node coordinates (or a pointer/index into the mesh's node array —
see the note under `Mesh` below about this exact decision), the element's own node-ID list,
and a cached quadrature rule (computing Gauss points fresh every call is wasted work — compute
once at construction, store as a member).

**Subclasses:** `Hex8Element`, `Tet4Element` — Step 1.2/1.3. Both must be swappable inside the
*same* `Mesh` (a mesh mixing hex and tet regions) because `Mesh` only ever stores
`unique_ptr<Element>`.

**Design check — a real tension in the plan:** `computeTangentStiffness` for a *nonlinear*
element must include both the **material stiffness** (from `Material::computeTangent`) and the
**geometric stiffness** (from the current stress state acting on the nonlinear strain-displacement
relationship — the σ:∇δu·∇u term). It's easy to implement only the material part, get something
that *looks* like it converges under small deformation, and only discover the missing geometric
term when Step 1.4's large-deformation benchmark either diverges or converges linearly instead
of quadratically. Bake a test for this into Step 1.2's acceptance criteria: verify tangent
stiffness by finite-difference perturbation of the residual (`K_ij ≈ (R_i(u+εe_j) - R_i(u))/ε`)
— this is the single most valuable unit test in the whole project and catches both missing terms
and sign errors.

> **Likely interview question:** "How do you validate a hand-derived tangent stiffness matrix?"
> — The finite-difference check above is exactly the expected answer.

---

## 3. `GlobalSystem` (not explicit in the plan, but required by its own signatures)

### Why this class needs to exist
The plan's own method signatures — `Mesh::assemble(GlobalSystem&, Material&)` and
`BoundaryCondition::apply(GlobalSystem&)` and `LinearSolver::solve(K_T, R) -> Δu` — all reference
a `GlobalSystem` that's never defined as its own class in the architecture diagram. Without it,
"assembly" has nowhere to assemble *into* except raw matrices passed around ad hoc, which
re-couples every class to Eigen's sparse types directly and makes it harder to swap in a
different backend representation later (relevant for Phase 3's CSR format on GPU).

```cpp
/// @brief Owns the global tangent stiffness matrix and residual vector for one
/// Newton iteration, plus the DOF bookkeeping needed to go from local element
/// contributions to global indices.
///
/// This is the "scratch space" that Mesh::assemble() writes into and that
/// BoundaryCondition::apply() modifies in place. Isolating it means the
/// sparse-matrix representation (triplets now, CSR-on-GPU later) can change
/// without touching Element, Material, or BoundaryCondition code.
class GlobalSystem {
public:
    explicit GlobalSystem(int numDofs);

    /// @brief Zero the residual and matrix triplet buffer before a new assembly pass.
    void reset();

    /// @brief Add a local element residual into the global residual at the given DOFs.
    void addResidual(const std::vector<int>& dofs, const Eigen::VectorXd& localR);

    /// @brief Add a local element tangent into the global matrix triplet buffer.
    void addTangent(const std::vector<int>& dofs, const Eigen::MatrixXd& localK);

    /// @brief Finalize triplets into a compressed sparse matrix (call once per iteration,
    /// after all elements have been assembled and before BCs are applied).
    void finalize();

    Eigen::SparseMatrix<double>& tangent();
    Eigen::VectorXd& residual();

private:
    int numDofs_;
    Eigen::VectorXd residual_;
    std::vector<Eigen::Triplet<double>> triplets_;
    Eigen::SparseMatrix<double> K_;
};
```

**Why `addTangent` buffers triplets instead of writing directly into the sparse matrix:** this is
the plan's own Step 1.3 acceptance criterion — "use `Eigen::Triplet` + `setFromTriplets`, not
per-coefficient insertion" — because repeated insertion into an already-compressed sparse matrix
is O(nnz) per insertion, while triplet accumulation + one `setFromTriplets` call is close to
O(nnz log nnz) total. This is worth stating explicitly as a design decision, not just a detail,
because it's the difference between assembly scaling linearly (the Step 1.3 acceptance criterion)
and scaling badly.

**Important — `tangent()` is a boundary, not a hand-off point:** `GlobalSystem::tangent()`
returns a raw `Eigen::SparseMatrix<double>&` because assembly and BC application are inherently
CPU/Eigen operations — that's fine. But `NewtonSolver` must **not** pass this reference straight
into `LinearSolver::solve()`. It has to wrap it in an `EigenSparseOperator` (§6a) first:
```cpp
EigenSparseOperator op(system.tangent());
Eigen::VectorXd du = linearSolver.solve(op, system.residual());
```
This one line is the entire seam between "assembly, which is always CPU/Eigen" and "linear
solve, which may be CPU or GPU" — skip it and `LinearSolver` silently depends on
`Eigen::SparseMatrix` again, undoing §6a below.

---

## 4. `Mesh`

### Why this class exists
`Mesh` is a pure orchestrator: it owns geometry and topology, and it drives the assembly loop,
but it contains zero physics itself. Every line inside `Mesh::assemble` should be sequencing and
bookkeeping — if you find yourself writing constitutive math or shape-function math inside
`Mesh`, that logic belongs in `Material` or `Element` instead.

```cpp
/// @brief Owns mesh topology (nodes, elements, DOF map) and drives assembly.
///
/// Mesh contains no physics: it loops over Elements and delegates all
/// mechanics to them polymorphically. This loop is written once and never
/// changes as new Element or Material subclasses are added.
class Mesh {
public:
    /// @brief Add an element, taking ownership. Heap-owned base-class pointers
    /// are what let a single mesh mix element types (hex + tet) transparently.
    void addElement(std::unique_ptr<Element> element);

    /// @brief Assemble global residual and tangent by looping over all elements
    /// and delegating to Element::computeResidual / computeTangentStiffness.
    void assemble(GlobalSystem& system, const Material& material,
                  const Eigen::VectorXd& globalDisplacement) const;

    const std::vector<Eigen::Vector3d>& nodeCoordinates() const;
    int numDofs() const;

private:
    std::vector<Eigen::Vector3d> nodeCoords_;
    std::vector<std::unique_ptr<Element>> elements_;
    // DOF map: node index -> global DOF indices (3 per node for 3D elasticity)
    std::vector<std::array<int,3>> dofMap_;
};
```

**Design check — a real gap in the plan:** the diagram passes a single `Material&` into
`Mesh::assemble`, meaning the whole mesh uses one material. But Step 1.5 introduces a second
material (`MooneyRivlinMaterial`) and real problems (a tire!) have multiple material regions
(rubber compound vs. steel belt reinforcement) in one mesh. Either that's explicitly out of
scope for this project (reasonable — say so in the README so it doesn't look like an oversight),
or `Element` should hold its *own* `Material*`/`Material&` reference set at construction rather
than receiving one externally at assembly time, with `Mesh::assemble` no longer taking a
`Material&` parameter at all. The latter is a small change now and a much bigger one to retrofit
after Phase 2 is built around the current signature — worth deciding before Step 1.3.

---

## 5. `BoundaryCondition` hierarchy

### Why this class exists
Boundary conditions modify the global system after assembly (row/column elimination, penalty
terms) and are logically independent of *how* the system was assembled. Making this its own
Strategy — rather than an `if` branch inside `NewtonSolver` — means Step 2.3's contact BC (which
breaks matrix symmetry, forcing a GMRES swap) is additive, and it's exactly what makes the
CG→GMRES story in Step 2.3 a clean demonstration rather than a hack.

```cpp
/// @brief Modifies the global system to enforce a constraint after assembly.
class BoundaryCondition {
public:
    virtual ~BoundaryCondition() = default;

    /// @brief Apply this BC in place to the assembled global system.
    virtual void apply(GlobalSystem& system) const = 0;
};

/// @brief Prescribes displacement at specific DOFs via row/column elimination.
class DirichletBC : public BoundaryCondition {
public:
    DirichletBC(std::vector<int> dofs, std::vector<double> prescribedValues);
    void apply(GlobalSystem& system) const override;
private:
    std::vector<int> dofs_;
    std::vector<double> values_;
};

/// @brief Penalty-based contact against a rigid plane; adds an asymmetric
/// stiffness/force contribution when penetration is detected.
class ContactBC : public BoundaryCondition {
public:
    ContactBC(std::vector<int> candidateNodeDofs, Eigen::Vector3d planeNormal,
              double planeOffset, double penaltyStiffness);
    void apply(GlobalSystem& system) const override;
private:
    std::vector<int> candidateDofs_;
    Eigen::Vector3d planeNormal_;
    double planeOffset_;
    double penalty_;
};
```

**Design check:** row/column elimination for Dirichlet BCs must be applied consistently at
*every* Newton iteration within a load step, using the *incremental* prescribed displacement
(not the total), or the load-stepping ramp in Step 1.4 will silently apply the full boundary
displacement in one shot regardless of load step. Worth a dedicated unit test: prescribe a
displacement ramp over N steps and assert intermediate deformed states are physically
intermediate, not just the final one.

---

## 6a. `LinearOperator` hierarchy — the fix that makes matrix-free CG and GPU swap both true

### Why this class exists (this is v2's "Fix 1," and it changes every class below it)
Without this class, `LinearSolver::solve` has to take a concrete matrix type, which forces an
impossible choice: either `K_T` is a bare `Eigen::SparseMatrix` — in which case going to GPU
means re-uploading it every single Newton iteration, defeating the entire point of Phase 3 — or
`ConjugateGradientSolver` reaches past `LinearSolver` and talks to `ComputeBackend` directly,
which breaks the Strategy separation between "how to iterate" (CG's job) and "where the mat-vec
runs" (`ComputeBackend`'s job). `LinearOperator` is the seam that lets both goals hold at once:
CG only ever calls `applyTo(x)`, and *what* `applyTo` does — Eigen SpMV on CPU, or a cached
device-resident SpMV on GPU — is entirely hidden inside the concrete `LinearOperator`.

```cpp
/// @brief Abstract "A*x without materializing A" operator.
///
/// This is the seam between assembly (always CPU/Eigen, via GlobalSystem)
/// and linear solve (may be CPU or GPU). LinearSolver and Preconditioner
/// depend only on this interface, never on a concrete matrix type or on
/// ComputeBackend — that's what makes "swap CpuBackend -> CudaBackend with
/// zero changes to CG" an actual, checkable claim instead of an aspiration.
class LinearOperator {
public:
    virtual ~LinearOperator() = default;

    /// @brief Compute y = A * x without ever exposing A itself.
    virtual Eigen::VectorXd applyTo(const Eigen::VectorXd& x) const = 0;

    /// @brief Matrix dimension (needed by CG/GMRES to size their work vectors).
    virtual int size() const = 0;
};

/// @brief Wraps a CPU-resident Eigen sparse matrix. Used by DirectSolver and
/// by ConjugateGradientSolver/GMRESSolver through Phase 2 (CPU-only).
class EigenSparseOperator : public LinearOperator {
public:
    explicit EigenSparseOperator(const Eigen::SparseMatrix<double>& A) : A_(A) {}
    Eigen::VectorXd applyTo(const Eigen::VectorXd& x) const override { return A_ * x; }
    int size() const override { return static_cast<int>(A_.rows()); }
private:
    const Eigen::SparseMatrix<double>& A_;
};

/// @brief Wraps a backend-resident matrix (Phase 3 onward). All device
/// residency and upload-caching logic lives here and in ComputeBackend —
/// ConjugateGradientSolver never knows this class exists.
class BackendOperator : public LinearOperator {
public:
    BackendOperator(ComputeBackend& backend, const DeviceCsrMatrix& A)
        : backend_(backend), A_(A) {}
    Eigen::VectorXd applyTo(const Eigen::VectorXd& x) const override {
        return backend_.spmv(A_, x);
    }
    int size() const override { return A_.numRows(); }
private:
    ComputeBackend& backend_;
    const DeviceCsrMatrix& A_;   // already resident on device; never re-uploaded per call
};
```

**Design check:** `EigenSparseOperator` stores `A_` by `const&`, not by value — it's a thin,
temporary view constructed fresh from `GlobalSystem::tangent()` each Newton iteration, not a
long-lived owner. Its lifetime must not outlive the `GlobalSystem` it wraps; in practice it's
constructed on the stack right before the `solve()` call and discarded immediately after (see
the `NewtonSolver` code in §11).

---

## 6b. `LinearSolver` hierarchy (Strategy pattern) — corrected to depend only on `LinearOperator`

### Why this class exists
This is the plan's clearest and most interview-relevant Strategy: `NewtonSolver` calls
`linearSolver.solve(op, R)` and never knows or cares whether `op` wraps a CPU matrix or GPU-
resident data, or whether the solve underneath is a dense LU factorization or 500 iterations of
preconditioned CG. The entire Phase 2 *and* Phase 3 narrative — "swap the linear solve, and later
the backend, without touching the Newton loop" — depends on this interface taking a
`LinearOperator&`, never a concrete matrix.

```cpp
/// @brief Strategy interface for solving A*du = R for the Newton update, where
/// A is only ever accessed through the abstract LinearOperator interface.
class LinearSolver {
public:
    virtual ~LinearSolver() = default;

    /// @brief Solve the linear system for the Newton increment.
    /// @param op Abstract operator representing the tangent matrix — never a
    /// concrete Eigen::SparseMatrix or GPU buffer type.
    /// @param R Residual vector (right-hand side).
    /// @return du, the Newton displacement increment.
    virtual Eigen::VectorXd solve(LinearOperator& op, const Eigen::VectorXd& R) = 0;

    /// @brief Diagnostics from the most recent solve (iteration count, residual
    /// history) — empty/trivial for direct solvers, populated for iterative ones.
    virtual SolverStats lastSolveStats() const = 0;
};

/// @brief Direct solve. Only ever used with an EigenSparseOperator in practice
/// (there's no such thing as a "matrix-free direct factorization"), but its
/// signature is identical to every other LinearSolver — it simply never
/// receives a BackendOperator in this project.
class DirectSolver : public LinearSolver {
public:
    Eigen::VectorXd solve(LinearOperator& op, const Eigen::VectorXd& R) override;
    SolverStats lastSolveStats() const override;
};

/// @brief Matrix-free, backend-agnostic Conjugate Gradient.
///
/// Note what is NOT a constructor parameter: ComputeBackend. CG never knows
/// which backend is in play — that knowledge is entirely contained inside
/// whichever LinearOperator it's handed at solve() time. This is the direct
/// fix for the coupling v2 explicitly warns against.
class ConjugateGradientSolver : public LinearSolver {
public:
    /// @param preconditioner Injected — the point Step 2.2 exploits to swap
    /// Identity/Jacobi/ILU with zero changes here.
    explicit ConjugateGradientSolver(Preconditioner& preconditioner,
                                      double tol = 1e-8, int maxIter = 1000);

    /// @brief Runs CG using only op.applyTo(x) — no branch, no cast, no
    /// knowledge of whether op is an EigenSparseOperator or BackendOperator.
    Eigen::VectorXd solve(LinearOperator& op, const Eigen::VectorXd& R) override;
    SolverStats lastSolveStats() const override;
private:
    Preconditioner& preconditioner_;
    double tol_;
    int maxIter_;
    SolverStats lastStats_;
};

class GMRESSolver : public LinearSolver { /* same shape as CG: op.applyTo(x) only */ };
```

**Properties worth calling out:** `ConjugateGradientSolver` takes `Preconditioner&` by reference
in its *constructor* (dependency injection, fixed for the solver's lifetime), but takes the
`LinearOperator&` as a parameter to `solve()`, not the constructor — because a fresh operator is
built every Newton iteration (wrapping that iteration's freshly-assembled tangent), while the
preconditioner strategy choice (Jacobi vs. ILU) is fixed for the whole simulation run. Different
lifetimes, different injection points — worth stating this distinction explicitly, since it's
easy to genericize both into "just inject everything at construction" and get it subtly wrong.

> **Likely interview question:** "Why does `Preconditioner` get constructor-injected but
> `LinearOperator` gets passed to `solve()`?" — Answer: they have different lifetimes. The
> operator is rebuilt every Newton iteration; the preconditioning *strategy* is a run-level
> choice. Injecting a per-iteration object at construction would force building a brand-new
> `ConjugateGradientSolver` every iteration for no reason.

**Design check:** `lastSolveStats()` isn't in the plan's original diagram but earns its place —
Step 2.2's entire deliverable is "iteration count and wall-clock time" benchmarking, and without
a defined way for `ConjugateGradientSolver` to report iteration count, that data has nowhere
principled to live except global variables or stdout scraping, both worth avoiding.

---

## 7. `Preconditioner` hierarchy (Strategy, injected into `LinearSolver`)

### Why this class exists
Separating "how to precondition" from "how to run CG" means Step 2.2's Jacobi→ILU comparison
touches zero lines of `ConjugateGradientSolver`. This is the second-clearest Strategy pattern in
the project and the one most directly relevant to a JD asking about "preconditioning."

```cpp
/// @brief Strategy interface: approximately solve M*z = r for a preconditioner M ≈ K_T.
class Preconditioner {
public:
    virtual ~Preconditioner() = default;

    /// @brief Given the tangent operator, build/update the preconditioner's
    /// internal state. Called once per Newton iteration, before apply() is
    /// called repeatedly by CG.
    virtual void setup(const LinearOperator& op) = 0;

    /// @brief Apply the preconditioner: z = M^{-1} r.
    virtual Eigen::VectorXd apply(const Eigen::VectorXd& r) const = 0;
};

class IdentityPreconditioner : public Preconditioner { /* z = r, setup() is a no-op */ };

class JacobiPreconditioner : public Preconditioner {
    // stores inverse diagonal, computed in setup() — see gap below
private:
    Eigen::VectorXd invDiag_;
};

class ILUPreconditioner : public Preconditioner {
    // wraps Eigen::IncompleteLUT<double>, built in setup() — CPU-only, see gap below
private:
    Eigen::IncompleteLUT<double> ilu_;
};
```

**Design check:** `setup()` is a separate call from `apply()` deliberately — the tangent changes
every Newton iteration (it's re-linearized each time), so the preconditioner must be rebuilt
every iteration too. If `apply()` alone triggered a rebuild, a naive `ILUPreconditioner`
implementation might recompute the incomplete factorization on *every CG iteration* instead of
once per Newton iteration — a correctness-preserving but catastrophically slow bug that's easy to
introduce and easy to miss in small test problems. Worth a benchmark assertion in Step 2.2:
preconditioner `setup()` call count should equal Newton iteration count, not CG iteration count.

**A real gap v2's own diagram doesn't resolve:** `setup()` above takes a `LinearOperator&`, kept
consistent with the "CG only touches `LinearOperator`" rule — but `LinearOperator`'s only
contract is `applyTo(x)`. `JacobiPreconditioner` needs the diagonal of the matrix, and
`ILUPreconditioner` needs the *entire* matrix to factor. Neither is derivable from `applyTo`
alone without expensive, indirect probing (applying the operator to unit basis vectors one at a
time — O(n) applyTo calls just to extract a diagonal). v2's diagram shows `Preconditioner::apply(r)
-> z` with no `setup()` at all, so this tension is inherited, not introduced here. It surfaces
concretely at Phase 3: once the tangent is GPU-resident behind a `BackendOperator`,
`JacobiPreconditioner::setup` has no CPU-side matrix to read a diagonal from. Two honest ways to
resolve it — decide before Step 3.3, not during it:
- **Widen the interface:** add an optional `diagonal()` method to `LinearOperator` (a
  `BackendOperator` can supply it cheaply — the diagonal of a CSR matrix is a fast device-side
  gather, not a full SpMV). This is what real matrix-free libraries (PETSc, Trilinos) do; "purely
  matrix-free" in practice usually means "no dense matrix materialized," not "the operator
  exposes literally nothing but `applyTo`."
- **Scope it out:** state explicitly in the README that `CudaBackend`-based CG runs
  unpreconditioned (`IdentityPreconditioner` only), and that Jacobi/ILU preconditioning is a
  CPU-path feature. This is a legitimate, defensible scope limit — just make it a stated decision
  in Phase 2, not a surprise discovered mid-Phase-3.

---

## 8. `ComputeBackend` hierarchy (Strategy/Bridge, swapped in Phase 3)

### Why this class exists
This is the interface that makes Phase 3 an *addition* rather than a rewrite. `spmv` (sparse
matrix-vector product) is CG's inner-loop bottleneck and the natural GPU port target (confirmed,
not assumed, by Step 3.1's profiling). Isolating it means `ConjugateGradientSolver` never
contains the word "CUDA" — and, per the §6a fix, `ConjugateGradientSolver` never even holds a
`ComputeBackend&` directly. The only class permitted to touch `ComputeBackend` is
`BackendOperator` (§6a). If you ever find `ComputeBackend` referenced from `LinearSolver`,
`Preconditioner`, or `NewtonSolver` code, that's the coupling v2's Fix 1 exists to prevent.

```cpp
/// @brief Strategy interface isolating *where* a sparse mat-vec product runs.
///
/// ConjugateGradientSolver calls backend_.spmv(...) and has no idea whether
/// that's a CPU loop or a CUDA kernel launch — this is what makes GPU support
/// an additive CudaBackend sibling, not a rewrite of CG.
class ComputeBackend {
public:
    virtual ~ComputeBackend() = default;

    /// @brief Compute y = A * x.
    virtual Eigen::VectorXd spmv(const Eigen::SparseMatrix<double>& A,
                                  const Eigen::VectorXd& x) const = 0;
};

class CpuBackend : public ComputeBackend {
    // trivial wrapper: return A * x using Eigen's existing SpMV
};

class CudaBackend : public ComputeBackend {
public:
    /// @brief Uploads A's CSR arrays and x to device once; subsequent spmv()
    /// calls in the same CG solve reuse device memory rather than re-uploading.
    Eigen::VectorXd spmv(const Eigen::SparseMatrix<double>& A,
                          const Eigen::VectorXd& x) const override;
private:
    mutable DeviceCsrMatrix d_A_;   // cached device-side CSR arrays
    mutable DeviceVector d_x_, d_y_;
};
```

**Design check — a real gap in the plan:** if `spmv` re-uploads `A` to device memory on *every*
call, host↔device transfer will dominate runtime and the GPU version will be slower than CPU at
every mesh size you can afford to test — which would falsely read as "GPU doesn't help here"
when the actual problem is a missing caching layer. `CudaBackend` needs to detect when `A` is
unchanged from the previous call (it is, for every CG iteration within one linear solve — only
`x` changes) and skip re-upload. This single detail is very likely the difference between Step
3.4's benchmark showing a real speedup or showing nothing.

> **Likely interview question:** "Your naive GPU version was slower than CPU — what did you do?"
> — This caching fix, plus the memory-coalescing work in Step 3.4, is exactly the expected
> answer, and it's a much stronger story than a GPU port that "just worked."

---

## 9. `NewtonSolver`

### Why this class exists
This is the orchestrator that ties every other Strategy together, and its entire job is to
know as little as possible about the concrete types underneath it — composed of references to
abstract bases, never inheriting from any of them.

```cpp
/// @brief Orchestrates the Newton-Raphson loop with load stepping.
///
/// Composed of (not inherited from) a Mesh, Material, LinearSolver, and set
/// of BoundaryConditions — every collaborator is touched only through its
/// abstract interface. Grep this class for concrete type names as a design
/// check: there should be none.
class NewtonSolver {
public:
    NewtonSolver(Mesh& mesh, Material& material, LinearSolver& linearSolver,
                 std::vector<std::reference_wrapper<BoundaryCondition>> boundaryConditions);

    /// @brief Run the full load-stepped Newton solve.
    /// @param numLoadSteps Number of increments to ramp load/prescribed displacement.
    /// @param residualTol Convergence tolerance on residual norm.
    /// @param dispTol Convergence tolerance on displacement increment norm.
    /// @param maxIterPerStep Newton iteration cap per load step (a solver that
    /// never converges should fail loudly, not loop forever).
    /// @return Final converged global displacement field.
    Eigen::VectorXd solve(int numLoadSteps, double residualTol, double dispTol,
                           int maxIterPerStep);

    /// @brief Convergence history for plotting: residual norm per iteration,
    /// grouped by load step. This is what Step 1.4's quadratic-convergence
    /// plot is generated from.
    const std::vector<std::vector<double>>& convergenceHistory() const;

private:
    Mesh& mesh_;
    Material& material_;
    LinearSolver& linearSolver_;
    std::vector<std::reference_wrapper<BoundaryCondition>> boundaryConditions_;
    std::vector<std::vector<double>> convergenceHistory_;
};
```

**Design check:** `boundaryConditions_` is `vector<reference_wrapper<BoundaryCondition>>` —
non-owning, but expressive about it (unlike a raw pointer, a `reference_wrapper` can't be null
and signals "I don't own this" in the type itself) — because `NewtonSolver` doesn't own the BCs;
the composition root (`main.cpp` or a driver/factory) does, and may need to hold onto them to
build a new `NewtonSolver` for the next load case. This is v2's Fix 2, correcting v1's raw
`BoundaryCondition*`, and it's consistent with Rule of Zero: `NewtonSolver` manages no resources
itself, so it needs no custom destructor, copy, or move logic at all — the compiler-generated
ones are correct by construction.

**Inside `solve()`, one Newton iteration looks like this** — note the `EigenSparseOperator`
constructed fresh each iteration, per the §6a / GlobalSystem note above:
```cpp
mesh_.assemble(system, material_, u);
for (auto& bc : boundaryConditions_) bc.get().apply(system);
system.finalize();

EigenSparseOperator op(system.tangent());     // wrap this iteration's tangent
Eigen::VectorXd du = linearSolver_.solve(op, system.residual());
u += du;
```
This is the only place in the whole codebase where an `EigenSparseOperator` gets constructed —
everywhere else, code talks to `LinearOperator&` and has no idea a concrete wrapper exists.

---

## 10. Factory / composition root

### Why this exists
Phase 0.5 asks for `createMaterial("neo-hookean", params)`-style construction so that swapping
implementations is a config change, not a recompile. This isn't a class in the inheritance sense
— it's a free function (or small `Factory` class) that maps strings/config values to concrete
types, and it's the *only* place in the codebase permitted to mention concrete class names like
`NeoHookeanMaterial` or `CudaBackend` outside of their own translation units and tests.

```cpp
/// @brief Constructs a concrete Material from a name and parameter map.
/// This is the one place in the codebase that is allowed to know about
/// every concrete Material subclass by name.
std::unique_ptr<Material> createMaterial(const std::string& type,
                                          const std::map<std::string, double>& params);

std::unique_ptr<LinearSolver> createLinearSolver(const std::string& type,
                                                  Preconditioner& preconditioner,
                                                  ComputeBackend& backend);
```

**Design check:** the Step 1.4 acceptance criterion ("grep `NewtonSolver` for concrete class
names — there should be none") should really be extended project-wide: grep *everything except
the factory functions and unit tests* for concrete class names. That's the actual test of
whether the Strategy pattern is real or cosmetic.

---

## Summary table

| Class | Role | Owns state? | Key collaborators |
|---|---|---|---|
| `Material` | Pure constitutive function | Params only, immutable | none |
| `Element` | Local kinematics + integration | Node IDs, quadrature rule | `Material` (by ref, per call) |
| `GlobalSystem` | Assembly scratch space | K triplets, R vector | none |
| `Mesh` | Topology + assembly loop | `vector<unique_ptr<Element>>`, nodes | `Element`, `GlobalSystem` |
| `BoundaryCondition` | Post-assembly constraint | BC-specific data | `GlobalSystem` |
| `LinearOperator` | "A·x" without materializing A | View of matrix / device buffer | `ComputeBackend` (`BackendOperator` only) |
| `LinearSolver` | Solve A·du=R via `LinearOperator` | Solver-specific state | `LinearOperator` (param), `Preconditioner` (CG, injected) |
| `Preconditioner` | Approximate inverse of A | Factorization/diagonal cache | `LinearOperator` (for `setup`) |
| `ComputeBackend` | Where SpMV runs | Device memory cache (CUDA) | wrapped by `BackendOperator` only |
| `NewtonSolver` | Orchestrate the Newton loop | Convergence history only | all of the above, by reference/`reference_wrapper` |

**Dependency direction, made explicit:** `ConjugateGradientSolver` depends on `LinearOperator`
and `Preconditioner` — never on `ComputeBackend`. `ComputeBackend` is only ever referenced inside
`BackendOperator`. If you draw this as a graph, `ComputeBackend` should be a leaf reachable only
through `BackendOperator → LinearOperator`, with no edge directly into any `LinearSolver`
subclass. That single missing/present edge is the concrete, checkable version of "is the Strategy
pattern real or cosmetic" for this part of the codebase.

---

## Open design questions to resolve before Phase 1 code (not just Phase 0.5 diagrams)

1. **Single material per mesh vs. per element** — flagged above under `Mesh`. Decide now; it's
   cheap now and expensive after Step 2 is built on top of the current signature.
2. **Where does the geometric stiffness term live?** — `Element::computeTangentStiffness` needs
   both material and geometric stiffness; make sure the finite-difference tangent check exists
   before you trust any benchmark built on top of it.
3. **Preconditioner rebuild frequency** — once per Newton iteration, not once per CG iteration;
   worth an explicit assertion/test, not just an implementation detail you get right by luck.
4. **`CudaBackend` device-memory caching** — without it, Step 3.4's benchmark risks showing no
   speedup for reasons unrelated to the actual GPU vs. CPU comparison you want to make.
5. **Preconditioner + matrix-free tension (§7)** — decide by Step 2.2, not Step 3.3, whether
   `LinearOperator` grows a `diagonal()` escape hatch or Jacobi/ILU are explicitly scoped to
   CPU-only. Either is fine; discovering the conflict mid-Phase-3 is not.

**Resolved in this revision (were open/incorrect in the first draft):**
- `LinearSolver::solve` now takes `LinearOperator&`, not a concrete matrix (§6a/§6b) — restores
  v2's Fix 1.
- `ConjugateGradientSolver` no longer holds `ComputeBackend&` — the backend is reachable only
  through whichever `LinearOperator` it's handed at `solve()` time.
- `NewtonSolver`'s boundary-condition list is `vector<reference_wrapper<BoundaryCondition>>`, not
  raw pointers — restores v2's Fix 2.
- Added the explicit `GlobalSystem::tangent() → EigenSparseOperator` wrap step, so the CPU/GPU
  seam has one clearly-named crossing point instead of being implicit.
