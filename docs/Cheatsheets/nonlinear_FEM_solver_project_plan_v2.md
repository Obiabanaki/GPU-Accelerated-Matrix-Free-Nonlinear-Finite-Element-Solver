# Project Plan: Nonlinear Hyperelastic FEM Solver (CPU → Krylov → GPU) — v2

**Goal:** A from-scratch C++ nonlinear FEM solver for large-deformation hyperelastic solids, with hand-rolled Krylov linear solvers, preconditioning, and a profiled/benchmarked GPU-accelerated bottleneck — built incrementally, tested at every stage, with a public commit history that tells the story.

**Total estimated time:** ~13–16 weeks part-time (revised from v1's 12–13; Phase 3 in particular was optimistic — see Phase 3 notes). Each step still has a hard stop-and-verify gate before moving on.

**Design philosophy (revised):** in v1, the full OOP architecture was locked in Phase 0.5 before any physics existed. That's premature abstraction — you can't design a stable `Material` interface before you know what `computeTangent` actually needs to return. **v2 splits this into two passes:** build Steps 1.1–1.2 with concrete classes first, *then* extract the abstract interfaces once the real seams are visible (new Phase 0.5 placement, below). The end architecture is the same one from v1 — it's just discovered instead of guessed, which is also a stronger interview answer than "I designed it all upfront."

---

## Phase 0: Setup (2–3 days)

Unchanged from v1.

**Tasks:**
- Create GitHub repo (e.g. `nonlinear-fem-solver`), MIT license, `.gitignore` for C++/CMake
- Set up CMake build system with a `src/`, `tests/`, `benchmarks/` structure
- Add Eigen (header-only) as a dependency
- Add GoogleTest or Catch2 for unit testing
- Write a minimal GitHub Actions workflow: build + run tests on every push
- Write a skeleton README stating the project's purpose and phased plan

**Acceptance criteria:** A stranger can `git clone`, build, and run tests successfully following only your README.

---

## Phase 0-pre: Concrete Neo-Hookean + Single Element (Weeks 1–2)

**Goal:** Get the material model and one element working as concrete, non-polymorphic classes first. This *is* the old Step 1.1 + 1.2 from v1, moved ahead of the architecture design on purpose — you cannot design a good interface for a class you haven't implemented yet.

### Constitutive model — with an explicit convention, not an implicit one

**Critique addressed:** v1's `Material::computeStress(F) -> S` never specified *which* stress (Cauchy? 1st PK? 2nd PK?) or what container represents the tangent. That's not a detail to defer — every `Element` that consumes `Material` depends on it, and a mismatch here is a silent correctness bug, not a compile error.

**Decision (fix this now, in code comments and in the README):**
- `computeStress(F)` returns the **2nd Piola–Kirchhoff stress S** (symmetric, defined in the reference configuration — the natural output of a strain-energy-density formulation like Neo-Hookean).
- `computeTangent(F)` returns the **material tangent modulus C in Voigt notation** (6×6 for 3D), i.e. `dS/dE` — not a raw rank-4 tensor. Voigt form is what `Hex8Element::computeTangentStiffness` will contract against the B-matrix, and it's what Eigen dense/sparse assembly expects.
- Document this mapping (F → S → Voigt C) in a single comment block at the top of `Material` once it exists, so nobody has to reverse-engineer units/order later.

**Tasks:**
- Implement `NeoHookeanMaterial` as a **concrete class, no base class yet** — just get it right
- Unit test against the analytical uniaxial tension solution; test F = Identity → stress exactly zero
- **New, non-negotiable: finite-difference check of the tangent.** Perturb F component-wise, recompute S, and confirm the numerical `dS/dF` (converted to Voigt) matches `computeTangent` to O(h) or O(h²) depending on scheme. This is the check that actually catches tangent bugs — matching the closed-form stress alone does *not* verify the tangent is consistent with it, and a wrong tangent doesn't show up as wrong stress, it shows up three weeks later as Newton silently losing quadratic convergence.

**Acceptance criteria:** Stress matches analytical uniaxial solution to 1e-8 across stretch ratios 0.5–3.0. **AND** FD tangent check passes to expected truncation-error tolerance across the same stretch range. Both criteria are required — pass one without the other and you have an undetected bug.

### Single-element Newton solve — still concrete

**Tasks:**
- Implement `Hex8Element` as a concrete class taking a concrete `NeoHookeanMaterial` (not yet a base-class reference)
- Implement a simple dense Newton solve (no `LinearSolver` abstraction yet — just call Eigen's `.solve()` directly)
- Solve the patch test: single element under prescribed uniform deformation

**Acceptance criteria:** Reproduces the exact analytical patch-test stress state.

**Why this reordering matters:** by the end of this phase you'll know, from direct experience, exactly what `Element` needs from `Material`, what `NewtonSolver` needs from `Element`, and what a linear solve step actually requires as inputs. That's the moment to extract interfaces — not before.

---

## Phase 0.5: OOP Architecture Design (3–5 days) — now *after* Phase 0-pre

**Goal:** Extract the abstract base classes and Strategy/Factory pattern from the two concrete implementations you just built, so every later phase (new material, new element, new linear solver, CPU→GPU swap) becomes "add a derived class" rather than "rewrite the core."

### Class architecture (mostly unchanged from v1, with two fixes)

```
Material (abstract base)
 ├── computeStress(F) -> S            [2nd Piola-Kirchhoff, pure virtual]
 ├── computeTangent(F) -> C_tangent   [Voigt 6x6 material tangent, pure virtual]
 ├── NeoHookeanMaterial : Material    (already built and FD-verified in Phase 0-pre)
 └── MooneyRivlinMaterial : Material

Element (abstract base)
 ├── computeResidual(u, Material&) -> Vector      [pure virtual]
 ├── computeTangentStiffness(u, Material&) -> Mat [pure virtual]
 ├── shapeFunctions(), gaussPoints()              [pure virtual]
 ├── Hex8Element : Element            (already built in Phase 0-pre)
 └── Tet4Element : Element

Mesh
 ├── owns std::vector<std::unique_ptr<Element>>
 ├── owns node coordinates, connectivity, DOF map
 └── assemble(GlobalSystem&, Material&)   [delegates to each Element]

BoundaryCondition (abstract base)
 ├── apply(GlobalSystem&)   [pure virtual]
 ├── DirichletBC : BoundaryCondition
 └── ContactBC : BoundaryCondition          (Phase 2.3, revised scope — see below)

LinearOperator (abstract base)        <-- NEW in v2, fixes a v1 gap (see note)
 ├── applyTo(x) -> y   [pure virtual, "A*x" without materializing A]
 ├── EigenSparseOperator : LinearOperator   (wraps an Eigen sparse matrix, used by DirectSolver/CG on CPU)
 └── BackendOperator : LinearOperator       (wraps a ComputeBackend-resident matrix, used from Phase 3 on)

LinearSolver (abstract base)          <-- Strategy pattern
 ├── solve(LinearOperator&, R) -> Δu   [pure virtual — CHANGED from v1's solve(K_T, R)]
 ├── DirectSolver : LinearSolver        (Phase 1, wraps Eigen dense/sparse LU)
 ├── ConjugateGradientSolver : LinearSolver   (Phase 2.1, matrix-free: only ever calls operator.applyTo(x))
 └── GMRESSolver : LinearSolver               (Phase 2.3)

Preconditioner (abstract base)        <-- Strategy pattern, injected into LinearSolver
 ├── apply(r) -> z   [pure virtual]
 ├── IdentityPreconditioner : Preconditioner
 ├── JacobiPreconditioner : Preconditioner
 └── ILUPreconditioner : Preconditioner

ComputeBackend (abstract base)        <-- Strategy/Bridge pattern, swapped in Phase 3
 ├── spmv(A, x) -> y   [pure virtual]
 ├── CpuBackend : ComputeBackend
 └── CudaBackend : ComputeBackend

NewtonSolver
 ├── owns Mesh&, Material&, LinearSolver&, vector<reference_wrapper<BoundaryCondition>>  [CHANGED, see note]
 ├── solve(loadSteps, tol, maxIter)
 └── orchestrates the Newton loop, calling into the above via interfaces only
```

**Fix 1 — `LinearOperator` abstraction (addresses a real design gap in v1):**
v1 had `LinearSolver::solve(K_T, R)` taking a concrete matrix, but also wanted `ConjugateGradientSolver` to be matrix-free and `ComputeBackend::spmv` to run on GPU-resident data. Those two goals conflict if `K_T` is a bare Eigen matrix: either you re-upload the matrix to the GPU every Newton iteration (defeats the point of Phase 3), or `ConjugateGradientSolver` has to know about `ComputeBackend` directly (breaks the Strategy separation). **Resolution:** `LinearSolver` operates on an abstract `LinearOperator` whose only contract is `applyTo(x) -> y`. On CPU this wraps an Eigen sparse matrix; from Phase 3 onward it wraps backend-resident data via `ComputeBackend::spmv`. `ConjugateGradientSolver` never sees the difference — which is what actually makes the Phase 3 "swap `CpuBackend`→`CudaBackend` with zero changes to CG" claim true, rather than aspirational.

**Fix 2 — `BoundaryCondition` ownership:**
v1 used `std::vector<BoundaryCondition*>` — raw, non-owning, unclear who's responsible for lifetime. This directly contradicts the project's own "modern C++, Rule of Zero over raw pointers" principle, and it's the kind of inconsistency an interviewer will notice immediately. **Resolution:** `std::vector<std::reference_wrapper<BoundaryCondition>>` if `NewtonSolver` doesn't own the BCs (ownership stays with `main`/factory), or `std::vector<std::unique_ptr<BoundaryCondition>>` if it should. Default to the reference_wrapper form unless a concrete reason to own emerges.

**Tasks:**
- Retroactively make `NeoHookeanMaterial` inherit from a newly-written `Material` abstract base — confirm existing Phase 0-pre tests still pass unmodified (this is the actual proof the abstraction is sound, not a fresh trivial-class demo)
- Same retrofit for `Hex8Element` → `Element`
- Write `LinearOperator`, `LinearSolver`, `Preconditioner`, `ComputeBackend`, `BoundaryCondition` as headers with pure virtual methods, no implementation yet beyond `DirectSolver`
- Add one trivial concrete class per remaining hierarchy (e.g. `NullMaterial`) to prove compilation
- Simple factory function (e.g. `createMaterial("neo-hookean", params)`)
- Document architecture diagram in `ARCHITECTURE.md`

**Acceptance criteria:** All Phase 0-pre tests pass unchanged after the retrofit to abstract base classes. A trivial end-to-end call (`NullMaterial` → dummy `Element` → `DirectSolver` via `LinearOperator`) compiles and runs.

---

## Phase 1: Nonlinear Hyperelastic Solver Core (Weeks 3–6)

*(Steps 1.1/1.2 from v1 are now done — folded into Phase 0-pre/0.5. Phase 1 picks up at mesh assembly.)*

### Step 1.3 — Multi-element mesh + sparse assembly

Unchanged from v1.

**Tasks:**
- `Mesh` owning `std::vector<std::unique_ptr<Element>>`
- Simple structured mesh generator (cube of hex elements)
- `Mesh::assemble(GlobalSystem&, Material&)`, polymorphic loop over `Element` base pointers
- Sparse **triplet** assembly (`Eigen::Triplet` + `setFromTriplets`)

**Acceptance criteria:** Assembly runtime scales roughly linearly with element count (first benchmark chart).

### Step 1.4 — Full Newton-Raphson solver with load stepping + cutback

**Critique addressed:** v1 ramped load in fixed increments with no fallback for non-convergence. A production-minded nonlinear solver — and the thing an interviewer will actually ask about — cuts the step and retries rather than failing outright.

**Tasks:**
- `DirichletBC : BoundaryCondition`
- `NewtonSolver`, composed of `Mesh&`, `Material&`, `LinearSolver&`, `vector<reference_wrapper<BoundaryCondition>>`
- Load stepping (ramp 0→100% over N increments)
- **New:** on non-convergence within `maxIter`, halve the step size and retry from the last converged state, up to a configurable minimum step size before declaring failure
- Convergence check (residual norm AND displacement increment norm)
- Benchmark problem: hyperelastic block under large uniaxial/shear deformation

**Acceptance criteria:** Solver converges quadratically near the solution (log(residual) vs. iteration plot). Final deformed shape is physically sensible (VTK → Paraview). Design check: grep `NewtonSolver` for concrete class names — none should appear, only base-class types. **New:** demonstrate the cutback logic firing on at least one deliberately over-large load step, and recovering to a converged solution.

### Step 1.5 — Near-incompressibility handling

Unchanged from v1.

**Tasks:**
- `MooneyRivlinMaterial : Material` — second genuine `Material` subclass
- Demonstrate locking at ν → 0.5 with standard displacement elements
- Selective reduced integration or mixed u-p formulation
- Re-run benchmark, show locking resolved

**Acceptance criteria:** Quantitative before/after comparison plot or table.

### Step 1.6 — Test suite and CI hardening

Unchanged from v1.

**Tasks:**
- Regression tests (patch test, single-element, benchmark problem) in CI
- Code coverage check
- Tag `v1.0-cpu-direct-solve`

**Acceptance criteria:** All tests pass in CI on a clean clone; release notes summarize v1.0 scope.

---

## Phase 2: Krylov Solvers + Preconditioning (Weeks 7–9)

### Step 2.1 — Hand-rolled Conjugate Gradient (matrix-free)

**Critique addressed:** with `LinearOperator` in place, CG should be written to never materialize or directly index into a matrix — only call `operator.applyTo(x)`. This is what actually makes it backend-agnostic later.

**Tasks:**
- `ConjugateGradientSolver : LinearSolver`, constructed with a `Preconditioner&` (default `IdentityPreconditioner`)
- Operates strictly through `LinearOperator::applyTo`, never touching a concrete matrix type
- Swap `DirectSolver` → `ConjugateGradientSolver` at the composition root; `NewtonSolver` unchanged
- Log iteration count and convergence history

**Acceptance criteria:** CG-based Newton loop reaches the same final deformed state as direct-solve (displacement diff below tolerance). Diff between CG-using and direct-solve-using driver code is one line.

### Step 2.2 — Preconditioning (Jacobi → ILU)

Unchanged from v1.

**Tasks:**
- `JacobiPreconditioner`, `ILUPreconditioner` (wrapping Eigen's `IncompleteLUT`)
- Benchmark all three preconditioners injected into the same `ConjugateGradientSolver` instance, across ≥3 mesh sizes, with zero changes to `ConjugateGradientSolver`

**Acceptance criteria:** Reproducible speedup, especially as mesh size and near-incompressibility-driven ill-conditioning increase. This is the single most JD-relevant chart.

### Step 2.3 — GMRES for non-symmetric cases, decoupled from contact

**Critique addressed:** v1 bundled a contact-mechanics implementation and a GMRES demonstration into one step. Contact (penalty search, gap function, potential ill-conditioning from penalty stiffness) is a hard problem in its own right and conflates two different sources of difficulty — if it slips, GMRES slips with it for no good reason. **Resolution:** demonstrate CG→GMRES motivation with a cheaper, still-legitimate source of non-symmetry — a follower load (pressure BC that rotates with the deforming surface, which introduces a non-symmetric load stiffness term) — and move full contact to an explicitly optional stretch phase.

**Tasks:**
- Implement a follower-load / deformation-dependent pressure BC that breaks tangent-matrix symmetry
- `GMRESSolver : LinearSolver` wrapping Eigen's GMRES, second sibling of `ConjugateGradientSolver`
- Swap at the composition root exactly as Step 2.1; `NewtonSolver` unchanged
- Document in the README why CG was no longer appropriate

**Acceptance criteria:** Solver handles the follower-load case; README explains the CG→GMRES decision with the actual matrix-symmetry argument.

### Step 2.3-stretch — Contact (optional, only if time permits)

**Tasks:** `ContactBC : BoundaryCondition`, simple penalty-based contact against a rigid plane, solved with the already-built `GMRESSolver`.

**Acceptance criteria:** Contact case converges; explicitly labeled in the README as a stretch addition, not required for the v2.0 tag.

### Step 2.4 — Tag release

**Tasks:** Tag `v2.0-krylov-preconditioned`, update README with benchmark charts.

**Acceptance criteria:** README "Results" section includes the Step 2.2 preconditioning chart.

---

## Phase 3: GPU Acceleration (Weeks 10–16)

**Critique addressed:** v1 budgeted this at 5 weeks (8–12), which is optimistic for a first CUDA project layered on top of C++ Newton/CG machinery — kernel debugging and memory-model issues routinely eat more time than planned. v2 widens the window to 7 weeks and makes the CuPy warm-up mandatory rather than optional if CUDA is new territory.

### Step 3.1 — Profile to find the real bottleneck

Unchanged from v1.

**Tasks:** Profile Phase 2 solver (wall-clock timers, then Nsight Systems if available) on a moderately large mesh; identify whether SpMV or assembly dominates.

**Acceptance criteria:** Quantitative justification for what gets ported.

### Step 3.2 — CuPy warm-up (now mandatory, not optional, if CUDA is new)

**Tasks:** Reimplement the CG SpMV step in CuPy against your solver's exported matrix; compare GPU vs. CPU NumPy timing across matrix sizes.

**Acceptance criteria:** Can explain host/device transfer cost, kernel launch overhead, and the GPU/CPU crossover point in your own words.

### Step 3.3 — CUDA SpMV kernel via `LinearOperator`/`ComputeBackend`

**Tasks:**
- `CpuBackend : ComputeBackend` (trivial Eigen SpMV wrapper) — confirms `ConjugateGradientSolver` was already written against `LinearOperator`/`ComputeBackend`, never against a raw matrix
- CSR-based SpMV kernel (one thread per row) inside `CudaBackend : ComputeBackend`
- Swap `CpuBackend` → `CudaBackend` at the composition root via a `BackendOperator`; `ConjugateGradientSolver` requires zero changes
- Validate GPU CG converges to the same solution as CPU CG

**Acceptance criteria:** Tolerance-matched agreement between CPU and GPU solves on the Phase 1 benchmark problem.

### Step 3.4 — Optimize and re-benchmark

Unchanged from v1.

**Tasks:** Improve memory coalescing (CSR reordering or warp-per-row); re-profile with Nsight; benchmark CPU-direct vs. CPU-CG+ILU vs. naive-GPU vs. optimized-GPU.

**Acceptance criteria:** Measurable speedup with a memory-bandwidth-based explanation, not just a number.

### Step 3.5 — Final packaging

Unchanged from v1.

**Tasks:** Comprehensive README (problem statement, architecture diagram, all benchmark charts, build/run instructions, future work); short demo or Paraview screenshots; tag `v3.0-gpu-accelerated`; update resume/LinkedIn.

**Acceptance criteria:** Someone with no prior context understands what was built, why, and sees quantitative evidence it works, without reading the code.

---

## Summary Timeline (revised)

| Phase | Weeks | Key Deliverable |
|---|---|---|
| 0 — Setup | 0.5 | CI-green empty solver |
| 0-pre — Concrete Neo-Hookean + single element | 1–2 | FD-verified material, patch-test-passing element, **no abstractions yet** |
| 0.5 — OOP architecture (extracted, not guessed) | +0.5 | Retrofit to abstract bases, `LinearOperator` + `BoundaryCondition` ownership fixed |
| 1 — Nonlinear hyperelastic core | 3–6 | Converged large-deformation solve with load-step cutback, locking handled, two `Material` subclasses |
| 2 — Krylov + preconditioning | 7–9 | Preconditioner benchmark chart, CG/GMRES via matrix-free `LinearOperator`, contact split off as stretch |
| 3 — GPU | 10–16 | Profiled, validated, benchmarked GPU speedup via `CudaBackend`, mandatory CuPy warm-up |

**Net changes from v1, in one place:**
1. Physics-before-architecture ordering (Phase 0-pre precedes Phase 0.5)
2. Explicit stress/tangent convention: 2nd PK stress, Voigt tangent
3. New `LinearOperator` abstraction so `LinearSolver`/`ComputeBackend` compose correctly
4. `BoundaryCondition` ownership fixed to `reference_wrapper`/`unique_ptr`, no raw pointers
5. Mandatory FD-check of the material tangent as a Phase 0-pre acceptance criterion
6. Adaptive load-step cutback added to Step 1.4
7. Contact split out of the GMRES step; follower-load used as the non-symmetry demonstration instead
8. Timeline extended to 13–16 weeks, with Phase 3 given 7 weeks instead of 5 and CuPy made mandatory

Each phase still ends with a tagged GitHub release and updated README — the project remains presentable even if you stop after Phase 1 or 2.
