# Linear Solvers & Preconditioners — Reference

This covers every `LinearSolver` and `Preconditioner` planned for the nonlinear FEM
solver — what each does, a small worked numeric example, and how GPU execution
plugs into the same code paths (Part 3).

## Where these fit in the bigger picture

```
NewtonSolver (outer, nonlinear loop)
  └─ each iteration: assemble tangent K_T and residual R, then solve K_T·du = R
        └─ LinearSolver::solve(op, R)      <- Part 1
              └─ (CG/GMRES only) Preconditioner::apply(r)   <- Part 2
                    └─ (Phase 3) op = BackendOperator, routed through ComputeBackend  <- Part 3
```

`NewtonSolver` never sees which `LinearSolver`, `Preconditioner`, or backend is
plugged in — that's the whole point of the Strategy pattern.

---

# Part 1 — Linear solvers (`fem::linalg::LinearSolver`)

All three solve the same equation — `A·du = R` for the Newton increment `du` — via
the abstract `LinearOperator` (never a raw matrix), so swapping one for another is a
one-line change at the composition root.

## `DirectSolver` — Phase 1 baseline

**What it does:** factorizes `A` exactly (LU decomposition) and solves by
forward/back substitution. No iteration, no approximation — exact up to
floating-point round-off.

**When it's used here:** the first working solver, before Krylov methods exist —
gets Phase 1's Newton loop running end-to-end without also debugging an iterative
method.

**Tradeoffs:** ✅ robust, no tuning; ❌ cost (sparse LU fill-in) scales badly with
problem size; ❌ needs the actual matrix — "matrix-free direct factorization" isn't
a real thing.

### Numeric example

Take the small SPD, tridiagonal system:
```
A = [4 1 0]     b = [1]
    [1 4 1]         [2]
    [0 1 4]         [3]
```
LU decomposition (Doolittle, `L` unit lower-triangular):
```
L = [1      0      0]     U = [4  1      0    ]
    [0.25   1      0]         [0  3.75   1    ]
    [0      0.2667 1]         [0  0      3.7333]
```
Forward-solve `L·y = b`: `y = [1, 1.75, 2.5333]`
Back-solve `U·x = y`: `x = [0.1786, 0.2857, 0.6786]`

Check: `A·x = [4(0.1786)+1(0.2857), 1(0.1786)+4(0.2857)+1(0.6786), 1(0.2857)+4(0.6786)]
            = [1.000, 2.000, 3.000]` ✓ matches `b` exactly.

Note this same `A` and its exact LU factors come back in the ILU example below —
worth comparing directly.

## `ConjugateGradientSolver` — Step 2.1

**What it does:** an iterative Krylov method for **symmetric positive-definite**
systems. Minimizes the residual along mutually "conjugate" search directions using
only matrix-vector products (`op.applyTo(x)`) — never the matrix's entries
directly. That's what makes it **matrix-free**: identical code whether `op` wraps a
CPU matrix or GPU-resident data (Part 3).

**Convergence:** iteration count scales with `sqrt(κ(A))`, the square root of the
condition number — why preconditioning matters so much for CG specifically (Part 2).

**Tradeoffs:** ✅ cheap per iteration; ✅ matrix-free, carries to GPU with zero code
changes; ❌ requires symmetry — feed it a non-symmetric matrix and it can silently
converge to the wrong answer; ❌ convergence rate is conditioning-dependent.

### Numeric example

Using `A = [[4,1],[1,3]]`, `b = [1,2]` (exact solution `x = [0.0909, 0.6364]`,
verify: `4(0.0909)+1(0.6364)=1.000`, `1(0.0909)+3(0.6364)=2.000` ✓):

| | `x` | `r = b - Ax` | `p` | `α` | notes |
|---|---|---|---|---|---|
| iter 0 | `[0, 0]` | `[1, 2]` | `[1, 2]` | `α₀ = (rᵀr)/(pᵀAp) = 5/20 = 0.25` | `Ap₀=[6,7]` |
| iter 1 | `[0.25, 0.5]` | `[-0.5, 0.25]` | `[-0.4375, 0.375]` (β₀=0.0625) | `α₁ = 0.3125/0.8594 = 0.3636` | `Ap₁=[-1.375,0.6875]` |
| iter 2 | **`[0.0909, 0.6364]`** | `≈[0,0]` | — | — | **exact solution** |

CG reaches the exact answer in exactly 2 iterations for this 2×2 system — the
**finite-termination property**: for an `n×n` SPD system, exact-arithmetic CG
converges in at most `n` iterations. In practice, round-off means real FEM problems
still benefit hugely from good preconditioning even though this bound technically
holds.

## `GMRESSolver` — Step 2.3

**What it does:** the general-purpose sibling of CG for **non-symmetric** systems.
Also matrix-free, but builds an orthogonal Krylov basis via Arnoldi iteration
instead of exploiting symmetry, and is periodically **restarted** (the `restart`
parameter) to bound memory growth.

**When it's used here:** motivated by a **follower load** BC (a pressure that
rotates with the deforming surface), which breaks tangent-matrix symmetry. The plan
deliberately avoided using contact for this — contact mixes in an unrelated,
harder-to-isolate source of ill-conditioning (penalty stiffness).

**Tradeoffs:** ✅ works on any non-singular matrix; ✅ same matrix-free contract as
CG; ❌ pricier per iteration (orthogonalization against a growing basis); ❌
`restart` needs tuning — too small hurts convergence, too large costs memory.

### Numeric example

Non-symmetric `A = [[3,1],[-1,2]]`, `b = [1,1]` (exact: `x = [0.1429, 0.5714]`).

Start `x₀=0`, `r₀=b=[1,1]`, `β=‖r₀‖=1.4142`, `v₁=r₀/β=[0.7071,0.7071]`.

Arnoldi step 1: `w=Av₁=[2.8284,0.7071]`, `h₁₁=v₁ᵀw=2.5`, orthogonalize:
`w−h₁₁v₁=[1.0607,−1.0607]`.

GMRES(1)'s least-squares solve gives `y₁=β/h₁₁=0.5657`, so:
```
x₁ ≈ v₁·y₁ = [0.4, 0.4]        r₁ = b − Ax₁ = [−0.6, 0.6]   (‖r₁‖=0.8485, down from 1.4142)
```
One iteration already cuts the residual by ~40%. Like CG, GMRES has finite
termination for an `n×n` system — a second Arnoldi step exactly recovers
`x = [0.1429, 0.5714]`, matching the direct solve.

## Quick comparison

| | `DirectSolver` | `ConjugateGradientSolver` | `GMRESSolver` |
|---|---|---|---|
| Matrix requirement | Any non-singular | Symmetric positive-definite | Any non-singular |
| Matrix-free? | No | Yes | Yes |
| Cost driver | Factorization fill-in | Iteration count × mat-vec cost | Iteration count × (growing) orthogonalization |
| Preconditioner? | Not applicable | Strongly recommended at scale | Strongly recommended at scale |
| Introduced | Phase 1 | Step 2.1 | Step 2.3 |
| Motivating case | Get Newton loop working | Symmetric hyperelastic tangent | Follower-load asymmetry |

---

# Part 2 — Preconditioners (`fem::linalg::Preconditioner`)

Only relevant to CG/GMRES. Solve the equivalent `M⁻¹A·x = M⁻¹b` for a cheap
approximation `M ≈ A`; if `M⁻¹A ≈ I`, the condition number collapses toward 1.
`setup()` must run **once per Newton iteration**, never once per CG iteration.

## `IdentityPreconditioner` — default, always available

**What it does:** `M = I`; `apply(r)` returns `r` unchanged.

**Numeric example:** for `r=[1,2]`, `z=apply(r)=[1,2]` — nothing happens. This is
the "no preconditioning" baseline every comparison chart is measured against.

## `JacobiPreconditioner` — Step 2.2

**What it does:** `M = diag(A)`. `setup()` reads the diagonal; `apply(r)` is one
elementwise divide.

**Numeric example:** using `A=[[4,1],[1,3]]` from the CG example, `M=diag(4,3)`,
`M⁻¹=diag(0.25, 0.3333)`. For `r=[1,2]`: `z = apply(r) = [0.25·1, 0.3333·2] = [0.25, 0.667]`.

Condition number check: `κ(A) = λmax/λmin`. `A`'s eigenvalues are `4.618` and
`2.382`, so `κ(A)=1.94`. `M⁻¹A = [[1, 0.25],[0.333, 1]]` has eigenvalues `1.288` and
`0.712`, so `κ(M⁻¹A)=1.81` — a modest improvement here, because this toy matrix is
already close to diagonally dominant. The real payoff shows up on FEM-scale
matrices under near-incompressibility, where off-diagonal coupling dominates and
Jacobi's improvement is much larger.

## `ILUPreconditioner` — Step 2.2

**What it does:** Incomplete LU (wraps `Eigen::IncompleteLUT`) — approximate
`L·U ≈ A`, keeping some off-diagonal structure while dropping small fill-in.

**Numeric example:** reuse the tridiagonal `A=[[4,1,0],[1,4,1],[0,1,4]]` from the
`DirectSolver` example. Because `A` is already tridiagonal, `ILU(0)` (the
zero-fill-in variant) produces **exactly the same `L`, `U`** shown there — no
approximation at all, since the sparsity pattern has no room for extra fill-in.
That makes `M⁻¹A = I` *exactly* for this case: a preconditioned CG solve on this
particular matrix would converge in effectively one step.

This is the illustrative point: `ILUPreconditioner` shines precisely when a
matrix's off-diagonal structure — like a Jacobi preconditioner would ignore, but a
real FEM tangent's local element-to-element coupling always has — is what's driving
the ill-conditioning. `JacobiPreconditioner` on this same tridiagonal `A` would use
only `diag(4,4,4)`, discarding the off-diagonal 1's entirely and converging
noticeably slower.

**Known limitation (Part 3 is exactly where this bites):** `ILUPreconditioner`
needs the **full matrix** to factor, not just `applyTo(x)` — CPU-only by
construction. See below.

## Quick comparison

| | `IdentityPreconditioner` | `JacobiPreconditioner` | `ILUPreconditioner` |
|---|---|---|---|
| Cost to build | None | Read the diagonal | Approximate factorization |
| Cost to apply | None | One elementwise divide | Two triangular solves |
| Captures off-diagonal coupling? | No | No | Yes |
| GPU-compatible (matrix-free)? | Yes | Needs `diagonal()` | No — needs the full matrix |
| Introduced | Phase 2.1 (default) | Step 2.2 | Step 2.2 |

---

# Part 3 — Solving on the GPU (Phase 3)

## What actually moves to the GPU

Only the sparse matrix-vector product (`SpMV`) — CG/GMRES's inner-loop bottleneck,
confirmed (not assumed) by Step 3.1's profiling. Everything else — the CG/GMRES
loop itself, dot products, the Newton loop, assembly — stays on the CPU. This is
why the architecture only needed one new seam:

```
ConjugateGradientSolver.solve(op, R)
        │  op.applyTo(x)   <-- the only thing CG ever calls on op
        ▼
BackendOperator::applyTo(x)  =  backend_.spmv(A, x)
        ▼
CudaBackend::spmv(A, x)      <-- the actual CUDA kernel, one thread per row
```

`ConjugateGradientSolver` never changes — it already only calls `op.applyTo(x)`,
so handing it a `BackendOperator` instead of an `EigenSparseOperator` is the entire
"port to GPU" story, by design (see the earlier `LinearOperator` discussion).

## CSR format, made concrete

`DeviceCsrMatrix` stores the matrix in **Compressed Sparse Row** form — three flat
arrays instead of a dense grid, which is what makes a GPU kernel practical (one
thread per row, no wasted work on zeros). Using the same tridiagonal
`A = [[4,1,0],[1,4,1],[0,1,4]]` from Parts 1–2:

```
values      = [4, 1,  1, 4, 1,  1, 4]     (nonzero entries, row by row)
col_indices = [0, 1,  0, 1, 2,  1, 2]     (which column each value belongs to)
row_ptr     = [0, 2, 5, 7]                (index into values[] where each row starts)
```
Row `i`'s nonzeros are `values[row_ptr[i] .. row_ptr[i+1]-1]`.

**SpMV via CSR**, computing `y = A·x` for `x = [1,1,1]` — row `i`'s dot product
only touches its own nonzero range:
```
row 0: values[0]*x[col[0]] + values[1]*x[col[1]]                = 4*1 + 1*1         = 5
row 1: values[2]*x[col[2]] + values[3]*x[col[3]] + values[4]*x[col[4]] = 1+4+1     = 6
row 2: values[5]*x[col[5]] + values[6]*x[col[6]]                = 1*1 + 4*1         = 5

y = [5, 6, 5]
```
Check against dense multiplication: `A·[1,1,1] = [4+1+0, 1+4+1, 0+1+4] = [5,6,5]` ✓.

`CudaBackend`'s planned kernel (Step 3.3) assigns exactly one GPU thread per row,
each independently walking its slice of `values`/`col_indices` — this CSR layout is
*why* that parallelization pattern works without threads stepping on each other.

## Why the naive port would be slower than CPU (and what fixes it)

Uploading `values`/`col_indices`/`row_ptr` to device memory has a fixed cost per
transfer. If `CudaBackend::spmv` re-uploaded `A` on **every single call**, and CG
calls `applyTo` once per iteration, you'd pay that transfer cost every iteration —
for a matrix that hasn't actually changed since the Newton iteration started (only
`x` changes between CG iterations, not `A`). At small-to-medium mesh sizes, that
transfer cost dwarfs the actual GPU compute time, and the "GPU" version loses to
plain CPU `EigenSparseOperator`.

The fix (flagged in `DeviceCsrMatrix`/`CudaBackend`'s TODOs): upload `A` **once per
Newton iteration** — cache the device-resident CSR arrays and only re-upload when
the matrix actually changes — so the per-CG-iteration cost is *only* the kernel
launch plus `x`'s (much smaller) transfer.

## Where a preconditioner fits on GPU — the gap from Part 2

This is where `ILUPreconditioner`'s CPU-only limitation from Part 2 becomes a real
decision point, not just a footnote: `LinearOperator`'s contract (`applyTo(x)`
only) has no way to hand a preconditioner the full matrix it needs to factor. Two
options, deliberately left open rather than guessed at:
- widen `LinearOperator` with a cheap `diagonal()` (a CSR diagonal gather is fast on
  GPU — this is what unlocks `JacobiPreconditioner` on `CudaBackend`, though not
  `ILUPreconditioner`, which needs more than the diagonal), or
- ship GPU-backed CG with `IdentityPreconditioner` only, as a documented, deliberate
  scope limit rather than a surprise discovered mid-implementation.

## What Step 3.1–3.2 establish before any of this is written

- **Step 3.1 (profiling):** confirm SpMV, not assembly, is actually the bottleneck
  worth porting — quantitative justification, not assumption.
- **Step 3.2 (CuPy warm-up):** reimplement just the SpMV step in CuPy against the
  solver's exported matrix, and compare GPU-vs-CPU timing across matrix sizes. The
  goal is to find the **crossover point** — the mesh size below which CPU wins
  outright because host↔device transfer and kernel-launch overhead dominate, and
  above which GPU wins. Being able to state that crossover point (and explain *why*
  it exists, in the terms above) is the explicit acceptance criterion for this step.

---

## The chart this all builds toward (Step 2.2's deliverable)

Run the same problem through `ConjugateGradientSolver` three times — once per
preconditioner — across a few mesh sizes and a few `kappa/mu` ratios, and plot
iteration count. Expected shape: `Identity`'s iteration count grows fastest as
either knob increases, `Jacobi` does better but still degrades under
near-incompressibility, and `ILU` stays comparatively flat. That comparison is
explicitly called the "single most JD-relevant chart" in the project plan.
