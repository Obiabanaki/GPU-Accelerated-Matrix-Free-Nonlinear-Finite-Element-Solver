# Nonlinear Hyperelastic FEM Solver

A from-scratch C++ nonlinear FEM solver for large-deformation hyperelastic
solids: hand-rolled Krylov linear solvers, preconditioning, and a
GPU-accelerated SpMV bottleneck via a Strategy/Bridge architecture.

See `nonlinear_FEM_solver_project_plan_v2.md` for the phased build plan and
`fem_solver_architecture.md` for the full class-by-class design rationale.

## Status

This is a **skeleton**: every class from the v2 architecture exists as a
compiling header/source pair with documented interfaces, but method bodies
are `TODO` stubs. Follow the project plan's phase order when filling them in
— in particular, `Material`/`Element` should be prototyped as concrete,
non-polymorphic classes first (Phase 0-pre) even though this skeleton already
shows them as abstract-base + subclass (the Phase 0.5 end state).

## Build

```bash
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build
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
| `mesh/` | `Mesh`, `GlobalSystem` | 0-pre, 1.3 |
| `bc/` | `BoundaryCondition`, `DirichletBC`, `ContactBC` | 1.4, 2.3-stretch |
| `linalg/` | `LinearOperator`, `EigenSparseOperator`, `BackendOperator`, `LinearSolver`, `DirectSolver`, `ConjugateGradientSolver`, `GMRESSolver`, `Preconditioner` + subclasses | 1, 2.1, 2.2, 2.3 |
| `backend/` | `ComputeBackend`, `CpuBackend`, `CudaBackend` | 3.3 |
| `solver/` | `NewtonSolver` | 1.4 |
| `factory/` | `createMaterial`, `createLinearSolver` | 0.5 |
