# C++ Project Structure — Cheat Sheet

## Core terms

| Term | Meaning | Example from this project |
|---|---|---|
| **Public API** | The contract a *user* of your library can depend on. Lives in headers under `include/`. Change it and everyone who includes it must adapt; change a `.cpp` body and nothing outside notices. | `include/fem/material/Material.hpp` — anyone can `#include` this and call `computeStress()`. |
| **`include/`** | Public headers (`.hpp`) only — declarations, no method bodies. What consumers of the library are allowed to see. | `Material.hpp` declares the interface, no math inside. |
| **`src/`** | Implementation (`.cpp`) only — method bodies. Compiled into the library, invisible to consumers. | `Material.cpp` has the actual Neo-Hookean stress formula. |
| **`fem_core`** | The CMake *library target* — not a class. `add_library(fem_core ...)` compiles all of `src/` once; `fem_demo`, `fem_tests`, `benchmark_assembly` each just link against it instead of recompiling the solver. | `target_link_libraries(fem_demo PRIVATE fem_core)` |
| **Header-only** | No `.cpp` at all — full implementation lives in the `.hpp`. Common for template libraries (Eigen) and small utility libraries. Costly here: every method-body change would force a full rebuild everywhere it's included. | Eigen itself; not used for this project's own classes. |
| **Template-heavy code** | Generic code (`std::vector<T>`-style) that mostly *must* live in headers, because the compiler needs the full definition at every call site to generate code per type `T`. | Eigen's matrix classes. This project's own classes (`Material`, `Element`, ...) are plain virtual-method classes, not templates. |
| **Utility library** | A small library of simple, mostly-stateless helpers (math functions, string formatting). Usually simple enough that header-only is fine — no real public/private boundary worth enforcing. | Not this project — it has real internal complexity, so `include/`+`src/` earns its keep. |
| **Helper header** | A header that exists only to share code *between your own `.cpp` files* — not part of the public API. Lives under `src/`, not `include/`, so outside code can't reach it. | e.g. a shared `computeJacobian()` used by both `Hex8Element.cpp` and `Tet4Element.cpp`, placed in `src/fem/element/internal/`. |
| **Grep-based design check** | Verifying an architectural rule by searching for what *shouldn't* appear, instead of manually re-reading code. Zero matches = rule upheld. | `grep -E "DirectSolver\|NeoHookeanMaterial" src/fem/solver/NewtonSolver.cpp` should return nothing — `NewtonSolver` should only reference abstract base types. |

## Project layout options, compared

| Structure | Layout | Best for | Tradeoff |
|---|---|---|---|
| **`include/` + `src/`** (used here) | Headers and `.cpp` in mirrored, separate trees | Libraries meant to be installed/consumed by other code; clear public/private boundary enforced by the filesystem itself | More directories to keep in sync; slight extra bookkeeping |
| **Colocated headers** | `.hpp` and `.cpp` side by side in one folder | Application code, fast day-to-day navigation ("everything about X in one place") | No structural signal for what's public vs. internal — must be enforced by convention |
| **Header-only** | Everything in `.hpp`, no `.cpp` | Template libraries, tiny utility libraries, easy drop-in reuse | Full rebuild cascades on every implementation change; not great for large/complex codebases |
| **Public/private split within one tree** | `include/` for public API + a private `internal/` folder inside `src/` for shared-but-not-public helpers | Projects that have grown helper code shared across multiple `.cpp` files | Slightly more structure to learn, but keeps the public API genuinely minimal |

## Why this project uses `include/` + `src/`

1. `CMakeLists.txt` already encodes it: `target_include_directories(fem_core PUBLIC include/)` — only `include/` is ever exposed to `fem_demo`/`fem_tests`/`benchmark_assembly`.
2. Grep-based design checks (e.g. "`NewtonSolver` should name no concrete classes") work *because* the tree enforces public/private — there's a real place to check.
3. Signals "this is a library," which matters for a portfolio project.
