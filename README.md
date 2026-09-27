# Nonlinear Hyperelastic FEM Solver

A from-scratch C++ nonlinear FEM solver for large-deformation hyperelastic
solids: hand-rolled Krylov linear solvers, preconditioning, and a
GPU-accelerated SpMV bottleneck via a Strategy/Bridge architecture.

See `docs/Cheatsheets/nonlinear_FEM_solver_project_plan_v2.md` for the phased
build plan and `docs/Cheatsheets/fem_solver_architecture.md` for the planned
design and class-by-class rationale.

## Status

This project is currently in a **facade / tracing pass**. The input reader,
structured mesh generation, face-to-DOF resolution, factory dispatch, and
the outer Newton load-step/iteration call sequence are implemented and
config-driven. The constitutive-material kernels and `Hex8Element` residual
and tangent routines are also implemented and tested in isolation.

The end-to-end solve is not physically complete yet. `Mesh::assemble` calls
the element routines with correctly sized zero local vectors but does not yet
gather the current global displacement or scatter element results into the
global system. `GlobalSystem`, boundary-condition application, iterative and
direct linear solvers, preconditioners, and CPU/CUDA SpMV remain trace
implementations that print the intended operation and return placeholders.
Consequently, `fem_demo` demonstrates the configured object graph and call
sequence; it does not produce a converged deformed shape.

**What's real right now:**
- `fem::io::loadSimulationConfig` — parses a JSON input file (see `examples/`).
- `fem::mesh::buildStructuredCubeMesh` — real structured-grid node/element
  generation, including a genuine hex-to-6-tet decomposition, so switching
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
- `NewtonSolver::solve`'s load-step/iteration loop **structure** — it
  genuinely loops and calls its collaborators in the intended order, but its
  convergence check is currently a zero-residual placeholder and it stops
  after the first iteration of each load step.

**What's trace-only (prints + placeholder return value):**
`Tet4Element::computeResidual/computeTangentStiffness`,
`GlobalSystem::addResidual/addTangent`, `BoundaryCondition::apply`,
`LinearSolver::solve`, `Preconditioner::setup/apply` (except
`IdentityPreconditioner`, which is genuinely trivial), and
`ComputeBackend::spmv`. `Mesh::assemble` is partially implemented: its
element-dispatch loop is real, but global displacement gathering and result
scattering are deferred.

Try it:
```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
./build/apps/fem_demo examples/example_problem.json       # hex8, neo-hookean, cg+jacobi
./build/apps/fem_demo examples/example_problem_alt.json   # tet4, mooney-rivlin, direct
```
Diff the two runs' output to see the same orchestration code create a
different object graph from the input file.

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
