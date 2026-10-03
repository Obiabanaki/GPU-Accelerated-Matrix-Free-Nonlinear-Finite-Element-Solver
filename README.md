# Nonlinear Hyperelastic FEM Solver

[![CI](https://github.com/Obiabanaki/GPU-Accelerated-Matrix-Free-Nonlinear-Finite-Element-Solver/actions/workflows/ci.yml/badge.svg)](https://github.com/Obiabanaki/GPU-Accelerated-Matrix-Free-Nonlinear-Finite-Element-Solver/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/ObiajuluM/GPU-Accelerated-Matrix-Free-Nonlinear-Finite-Element-Solver)](https://github.com/Obiabanaki/GPU-Accelerated-Matrix-Free-Nonlinear-Finite-Element-Solver/releases)
[![codecov](https://codecov.io/gh/Obiabanaki/GPU-Accelerated-Matrix-Free-Nonlinear-Finite-Element-Solver/branch/main/graph/badge.svg)](https://codecov.io/gh/Obiabanaki/GPU-Accelerated-Matrix-Free-Nonlinear-Finite-Element-Solver)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://Obiabanaki.github.io/GPU-Accelerated-Matrix-Free-Nonlinear-Finite-Element-Solver/)
[![License](https://img.shields.io/github/license/Obiabanaki/GPU-Accelerated-Matrix-Free-Nonlinear-Finite-Element-Solver)](LICENSE)

A from-scratch C++ nonlinear FEM solver for large-deformation hyperelastic
solids: hand-rolled Krylov linear solvers, preconditioning, and a
GPU-accelerated SpMV bottleneck via a Strategy/Bridge architecture.

See `docs/Cheatsheets/nonlinear_FEM_solver_project_plan_v2.md` for the phased
build plan and `docs/Cheatsheets/fem_solver_architecture.md` for the planned
design and class-by-class rationale.

## Status

The project now has a working load-stepped Newton solve for Hex8 elements with
the implemented hyperelastic materials, Dirichlet displacement constraints,
and the sparse direct linear solver. Global element residuals/tangents are
assembled, and `fem_demo` prints the converged nodal displacement vector.
Iterative solvers, non-identity preconditioners, Tet4 physics, contact, and
CPU/CUDA SpMV are still trace implementations. Failed load increments are
halved and retried, with a bounded number of cutbacks.

**What's real right now:**
- `fem::io::loadSimulationConfig` — parses a JSON input file (see `examples/`).
- `fem::mesh::buildMesh` — dispatches to structured cube or 90-degree
  thick-cylinder-sector Hex8 meshes. Cube generation includes a genuine
  hex-to-6-tet decomposition, so switching
  `mesh.element_type` between `"hex8"` and `"tet4"` produces an actually
  different mesh (48 tets vs. 8 hexes for a 2x2x2 grid). The current pipeline
  tests verify the generated node grid, face-node sets, and supported element
  configurations; element-count introspection is not yet exposed by `Mesh`.
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
- `Mesh::assemble` and `GlobalSystem` — gather/scatter element data and
  assemble the global sparse tangent and residual.
- `DirichletBC` and `PressureBC` — enforce prescribed displacement increments
  or follower pressure loads, including the pressure-load tangent.
- `DirectSolver` and `NewtonSolver::solve` — perform sparse LU Newton updates
  with residual and displacement-increment convergence checks, plus bounded
  load-increment cutback.

**What's trace-only (prints + placeholder return value):**
`Tet4Element::computeResidual/computeTangentStiffness`, `ContactBC::apply`,
CG/GMRES `LinearSolver::solve`, Jacobi/ILU `Preconditioner::setup/apply`, and
`ComputeBackend::spmv`. `IdentityPreconditioner` is a real identity
operation. `DirectSolver`, `DirichletBC`, global assembly, and Newton cutback
are implemented.

Try it:
```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
./build/apps/fem_demo examples/example_problem.json       # hex8, neo-hookean, direct
./build/apps/fem_demo examples/analytical_newton.json     # validated affine Hex8 solution
./build/apps/fem_demo examples/cylinder_inflation.json    # pressurized thick-cylinder sector
```
`analytical_newton.json` prescribes the exact affine field
`u_x = 0.1 x, u_y = u_z = 0` on one Hex8; its integration test checks every
returned nodal DOF. The nonlinear spring test checks Newton's free-DOF
equilibrium against the exact root `u = 1`. The Tet4 alternate input is retained
to exercise config/factory selection, but does not converge until Tet4 residual
and tangent physics are implemented.

### Cylinder Inflation Reference

`cylinder_inflation.json` models a 90-degree sector of a thick cylinder in
plane strain. Symmetry conditions are applied on the radial cut faces, both
axial faces have zero axial displacement, and positive follower pressure is
applied on the inner wall. The continuum reference in
`NewtonSolver.CylinderInflationMatchesRadialEquilibriumReference` assumes
axisymmetric motion `r=r(R)`, with stretches `lambda_r=dr/dR`,
`lambda_theta=r/R`, and `lambda_z=1`. For the same compressible Neo-Hookean
energy used by the FE solve, the radial nominal stresses obey

```text
P_RR = lambda_r * S_RR
P_TT = lambda_theta * S_TT
dP_RR/dR = (P_TT - P_RR)/R
P_RR(inner_radius) = -pressure * lambda_theta(inner_radius)
P_RR(outer_radius) = 0
```

The test solves this radial boundary-value problem by shooting with RK4, then
compares radial displacement at each mesh radius along the sector centerline
with a 3 mm tolerance. This is a semi-analytical continuum reference, not a
closed-form solution; using the same compressible strain-energy law avoids
confounding discretization error with the incompressible limit. It validates
the pressure load, plane-strain constraints, assembly, and Newton equilibrium
as one solver path. For the current 12x8 sector mesh (inner radius 1 m, outer
radius 2 m, mu=1, kappa=100, inner pressure 0.1), all five load steps converge.
The measured comparison is:

| Reference radius (m) | FE radial displacement (m) | Radial reference (m) | Absolute error (m) |
|---:|---:|---:|---:|
| 1.00 | 0.070941 | 0.073690 | 0.002749 |
| 1.50 | 0.048342 | 0.050239 | 0.001897 |
| 2.00 | 0.036661 | 0.038105 | 0.001444 |

The maximum error over the 13 radial nodes is 2.749 mm.
The full test suite passes 46 tests; the two skipped tests are the existing
unimplemented Tet4 physics checks.

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

## Debugging

Build with debug symbols before starting a debugging session:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
```

### VS Code

Install the Microsoft C/C++ extension and start the `Debug fem_demo`
configuration with F5. Set a source breakpoint by clicking the gutter beside
the line, then restart debugging after changing breakpoints if the program has
already exited. A normal exit with code 0 means the program completed; it does
not mean a breakpoint was hit. The `setupCommands` enabling GDB pretty-printing
must be inside the selected `cppdbg` configuration in `.vscode/launch.json`.

### GDB and values

You can also launch the demo directly in GDB:

```bash
gdb --args ./build/apps/fem_demo examples/example_problem.json
```

At the GDB prompt, set a breakpoint by file and line or by function, then run:

```gdb
break apps/main.cpp:LINE
break fem::mesh::buildStructuredCubeMesh
run
info locals
print config.material.params
```

Replace `LINE` with the desired source line. In VS Code, use the **Variables**
and **Watch** panes or enter GDB commands in **Debug Console** with `-exec`,
for example `-exec p config.material.params`. Inspect a value only while paused
and while its variable is in scope. `info locals` lists locals, `print EXPR`
evaluates an expression, and `ptype EXPR` shows its type.

The debugger may expose implementation details instead of friendly values:

- A `std::unique_ptr` such as `built.mesh` owns an object; use `built.mesh->...`
  in C++ expressions. `built.mesh.get()` prints the pointee address, while
  `*built.mesh.get()` asks GDB to print the object.
- Private members can often be inspected in **Variables** or with GDB when
  debug symbols are present, even though normal source code outside the class
  cannot access them.
- If a `std::vector` or `std::map` shows fields such as `_M_start`, `_M_finish`,
  or `_M_t`, those are libstdc++ storage internals. Enable GDB pretty-printing,
  then expand the container or inspect an entry, e.g. `-exec p values[0]` or
  `-exec p config.material.params`. For a map, `first` is a key and `second`
  is its value. Avoid `map[key]` just to inspect it because it can insert a
  missing key; use `map.at(key)` in source code instead.
- GDB may say `Cannot evaluate function -- may be inlined` when Watch or Debug
  Console tries to call an inline method. Instead, capture the result in a
  source local and break on a later line. For example, after `built` is made:

  ```cpp
  const auto& coords = built.mesh->nodeCoordinates();
  const double x0 = coords[0][0];
  ```

  Inspect `x0` in **Variables**. The same technique works for method results
  and Eigen components that the debugger cannot evaluate directly.

Set useful breakpoints in `fem::NewtonSolver::solve`, `fem::Mesh::assemble`,
`fem::Hex8Element::computeResidual`, or
`fem::Hex8Element::computeTangentStiffness`. To debug a test, use
`./build/tests/fem_tests --gtest_filter="Hex8Element.*"` as the program and
argument in GDB or VS Code.

## Documentation

Public classes and methods are documented with Doxygen comments. Generate
browsable HTML with either:

- **Directly with doxygen** (needs `doxygen` on your `PATH`):
  ```bash
  doxygen Doxyfile
  # open docs/html/index.html
  ```
- **Via the CMake `docs` target** (available whenever CMake's `find_package(Doxygen)`
  locates a doxygen install; configure once with `cmake -S . -B build`):
  ```bash
  cmake --build build --target docs
  # open docs/html/index.html
  ```
  This target has no tracked outputs, so it reruns Doxygen and regenerates
  `docs/html` every time it is invoked.

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
