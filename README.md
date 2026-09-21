# Nonlinear Hyperelastic FEM Solver

A from-scratch C++ nonlinear FEM solver for large-deformation hyperelastic
solids: hand-rolled Krylov linear solvers, preconditioning, and a
GPU-accelerated SpMV bottleneck via a Strategy/Bridge architecture.

See `nonlinear_FEM_solver_project_plan_v2.md` for the phased build plan and
`fem_solver_architecture.md` for the full class-by-class design rationale.

## Status

This project is currently in a **facade / tracing pass**: the orchestration
layer — reading an input file, generating mesh topology, and wiring up
concrete `Material`/`Element`/`BoundaryCondition`/`LinearSolver`/
`Preconditioner` objects via the factories — is genuinely implemented and
config-driven. Most of the remaining FEM math (`solve`, `apply`, `spmv`,
...) is still in **trace mode**: each method prints what it would do and
returns a placeholder value instead of computing. Running `fem_demo`
produces a full, readable log of exactly which objects got created and
which methods got called, in order, for a given input file — the
`Hex8Element`/`NeoHookeanMaterial`/`MooneyRivlinMaterial` combination now
also computes a real residual/tangent along the way, but the surrounding
solve (`NewtonSolver`/`LinearSolver`/`Preconditioner`) is still traced, so
no deformed shape comes out the other end yet.

**What's real right now:**
- `fem::io::loadSimulationConfig` — parses a JSON input file (see `examples/`).
- `fem::mesh::buildStructuredCubeMesh` — real structured-grid node/element
  generation, including a genuine hex-to-6-tet decomposition, so switching
  `mesh.element_type` between `"hex8"` and `"tet4"` produces an actually
  different mesh (48 tets vs. 8 hexes for a 2x2x2 grid — verified in
  `tests/test_pipeline.cpp`).
- `fem::factory::*` — every factory function dispatches on the config
  string to a real concrete class.
- `NeoHookeanMaterial`/`MooneyRivlinMaterial` — real, closed-form
  compressible hyperelastic `computeStress`/`computeTangent`/
  `strainEnergy`, FD-verified in `tests/test_material.cpp` (see each
  class's `.cpp` file comment for the formulas and documented
  limitations).
- `Hex8Element` — real total-Lagrangian `computeResidual`/
  `computeTangentStiffness` (shape functions and the Gauss rule were
  already real, pure reference-element geometry), FD-verified and
  patch-tested in `tests/test_element.cpp`.
- `Tet4Element`'s shape functions and Gauss quadrature rule — pure
  reference-element geometry, not physics; its residual/tangent are still
  trace-only (see below).
- `NewtonSolver::solve`'s load-step/iteration loop **structure** — it
  genuinely loops and genuinely calls its collaborators in the right order;
  the collaborators themselves are traced.

**What's trace-only (prints + placeholder return value):**
`Tet4Element::computeResidual/computeTangentStiffness`,
`GlobalSystem::addResidual/addTangent`, `BoundaryCondition::apply`,
`LinearSolver::solve`, `Preconditioner::setup/apply` (except
`IdentityPreconditioner`, which is genuinely trivial), `ComputeBackend::spmv`.

Try it:
```bash
cmake -S . -B build -G Ninja && cmake --build build
./build/apps/fem_demo examples/example_problem.json       # hex8, neo-hookean, cg+jacobi
./build/apps/fem_demo examples/example_problem_alt.json   # tet4, mooney-rivlin, direct
```
Diff the two runs' output to see the same orchestration code create a
different object graph purely from the input file.

## Build

```bash
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build
```

## Tests

Unit tests live in `tests/` (one file per class/behavior) and are built as
the `fem_tests` target via GoogleTest.

Build and run everything:
```bash
cmake --build build --target fem_tests
ctest --test-dir build --output-on-failure
```

Run the test binary directly — useful for filtering to a subset by name
(GoogleTest's `--gtest_filter` supports glob patterns):
```bash
./build/tests/fem_tests                              # run everything
./build/tests/fem_tests --gtest_filter="Hex8Element.*" # only Hex8Element tests
```

## Documentation

Every class and public method carries doxygen comments (`@file`, `@brief`, `@param`, `@return`). Generate browsable HTML with:

```bash
doxygen Doxyfile
# open docs/html/index.html
```

## Layout

```
include/fem/    Public headers, one subfolder per architectural module
src/fem/        Implementation (.cpp) for every concrete class
src/fem/internal/  Private helper headers (e.g. VoigtUtil.hpp) — reachable
                only from fem_core's own .cpp files, not by consumers
examples/       Sample input files for fem_demo
apps/           main.cpp — composition root; the only place concrete
                classes are named outside their own translation units
tests/          GoogleTest unit tests, one file per class/behavior
benchmarks/     Standalone timing harnesses (assembly, CG, SpMV)
```

## Module map

| Folder | Classes | Plan phase |
|---|---|---|
| `material/` | `Material`, `NeoHookeanMaterial`, `MooneyRivlinMaterial` | 0-pre, 1.5 |
| `element/` | `Element`, `Hex8Element`, `Tet4Element` | 0-pre, 1.3 |
| `mesh/` | `Mesh`, `GlobalSystem`, `MeshBuilder` | 0-pre, 1.3 |
| `bc/` | `BoundaryCondition`, `DirichletBC`, `ContactBC` | 1.4, 2.3-stretch |
| `linalg/` | `LinearOperator`, `EigenSparseOperator`, `BackendOperator`, `LinearSolver`, `DirectSolver`, `ConjugateGradientSolver`, `GMRESSolver`, `Preconditioner` + subclasses | 1, 2.1, 2.2, 2.3 |
| `backend/` | `ComputeBackend`, `CpuBackend`, `CudaBackend` | 3.3 |
| `solver/` | `NewtonSolver` | 1.4 |
| `factory/` | `createMaterial`, `createElement`, `createBoundaryCondition`, `createPreconditioner`, `createLinearSolver` | 0.5 |
| `io/` | `SimulationConfig`, `loadSimulationConfig` | facade pass |
