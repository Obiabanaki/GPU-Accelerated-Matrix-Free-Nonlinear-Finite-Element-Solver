# CMake Cheat Sheet

Reference for every CMake concept and command used in `nonlinear-fem-solver`'s build system.

## The file tree, in build order

```
CMakeLists.txt              root: global settings, fetches Eigen, add_subdirectory() into the rest
├── src/CMakeLists.txt      builds fem_core (the library)
├── apps/CMakeLists.txt     builds fem_demo (executable, links fem_core)
├── tests/CMakeLists.txt    fetches GoogleTest, builds fem_tests, registers with ctest
└── benchmarks/CMakeLists.txt   builds benchmark_assembly (executable, links fem_core)
```
`add_subdirectory(X)` in a parent file is what tells CMake to go read `X/CMakeLists.txt` next — this is the whole tree structure.

## Core commands, one line each

| Command | What it does |
|---|---|
| `cmake_minimum_required(VERSION 3.20)` | Minimum CMake version this project needs. |
| `project(name LANGUAGES CXX)` | Names the project; declares which compilers to configure (CXX = C++; CUDA added separately, conditionally). |
| `set(VAR value)` | Sets a CMake variable — e.g. `CMAKE_CXX_STANDARD 17` requires C++17. |
| `option(NAME "help text" ON/OFF)` | Defines a toggleable build switch, settable from the command line: `-DFEM_BUILD_TESTS=OFF`. |
| `if(...) ... endif()` | Conditional logic — e.g. only `enable_language(CUDA)` when `FEM_ENABLE_CUDA` is on. |
| `enable_language(CUDA)` | Turns on CUDA as a compiler CMake knows about; only called when needed, so machines without `nvcc` aren't affected. |
| `include(FetchContent)` | Loads CMake's built-in dependency-fetching module. |
| `FetchContent_Declare(name GIT_REPOSITORY ... GIT_TAG ...)` | Registers where to get a dependency and which version/tag. |
| `FetchContent_MakeAvailable(name)` | Actually clones + configures the dependency, exposing its CMake targets (e.g. `Eigen3::Eigen`). |
| `add_subdirectory(dir)` | Processes `dir/CMakeLists.txt` as part of this build. |
| `add_library(name ...)` | Compiles a list of `.cpp` files into a library target (static by default). |
| `add_executable(name ...)` | Compiles a list of `.cpp` files into a runnable binary. |
| `target_include_directories(target VISIBILITY dir)` | Adds `dir` to the compiler's search path for `target` (and downstream, if `PUBLIC`). |
| `target_link_libraries(target VISIBILITY dep)` | Links `target` against `dep`; also propagates `dep`'s `PUBLIC` include paths/flags if `VISIBILITY` is `PUBLIC`. |
| `target_compile_features(target VISIBILITY cxx_std_17)` | Requires a specific C++ standard for just this target (more granular than the global `set(CMAKE_CXX_STANDARD ...)`). |
| `target_sources(target VISIBILITY file.cpp)` | Adds a source file to an *already-declared* target — used conditionally for `CudaBackend.cpp`. |
| `set_source_files_properties(file PROPERTIES LANGUAGE CUDA)` | Forces one specific file to compile with the CUDA compiler instead of the default C++ one. |
| `enable_testing()` | Turns on CTest for this project; must come before `add_subdirectory(tests)`. |
| `include(GoogleTest)` + `gtest_discover_tests(target)` | Runs the compiled test binary once at build time to ask "what tests do you have?", then registers each one individually with `ctest`. |

## `PUBLIC` / `PRIVATE` / `INTERFACE` — visibility keywords

| Keyword | Meaning | Example |
|---|---|---|
| `PRIVATE` | Only this target needs it; nothing downstream inherits it. | `fem_demo` links `fem_core` `PRIVATE` — nothing links `fem_demo` itself, so there's no "downstream" to propagate to. |
| `PUBLIC` | This target needs it, AND anyone linking this target needs it too. | `fem_core`'s `include/` dir and its Eigen dependency are `PUBLIC` — because `Material.hpp` (a public header) uses `Eigen::Matrix3d` in its signature, so anyone including it needs Eigen too. |
| `INTERFACE` | This target doesn't need it itself, but anyone linking it does. | Used for header-only libraries with no `.cpp` to compile — e.g. how Eigen represents itself internally. |

**Rule of thumb:** ask "does *this target's own `.cpp` files* need it?" and "does *code that links this target* need it?" — pick the keyword that matches both answers.

## Library target types

| Type | CMake syntax | File produced | Key tradeoff |
|---|---|---|---|
| **Static** (default, used here) | `add_library(name ...)` or `add_library(name STATIC ...)` | `.a` / `.lib` | Code copied into every executable that links it; no runtime file to lose track of; larger binaries if linked by many executables. |
| **Shared / dynamic** | `add_library(name SHARED ...)` | `.so` / `.dylib` / `.dll` | Code lives in one file, loaded at runtime by multiple executables; smaller binaries, but the `.so` must be found at runtime (`LD_LIBRARY_PATH` etc.) — a common source of deployment bugs. |
| **Header-only / `INTERFACE`** | `add_library(name INTERFACE)` | none — no `.cpp` to compile | Simplest to consume (just `#include`), but every method-body change forces a rebuild everywhere it's included. Used by Eigen. |
| **Object library** | `add_library(name OBJECT ...)` | `.o` files, not bundled | Compiles once, reusable inside both a static and shared build of the same code without double-compiling. Niche; not used in this project. |
| **C++20 modules** | (language feature, not a CMake library type) | — | Newer alternative to header `#include`s, faster/more isolated compiles. Not applicable — this project targets C++17. |

## Dependency management in C++ (there's no single official one)

| Tool | What it is | Used here? |
|---|---|---|
| **CMake `FetchContent`** | Downloads a dependency's source (via git) and builds it as part of your build. Simple, zero extra tools needed, but slow (everyone rebuilds from source) and no real version registry. | ✅ Yes — for Eigen and GoogleTest. |
| **Conan** | A real package manager: central index, prebuilt binaries when available, version resolution. | No, but a natural upgrade later. |
| **vcpkg** | Microsoft's package manager; integrates tightly with CMake/Visual Studio. | No. |
| **System package manager** (`apt`, `brew`, `vcpkg` as an OS-level tool) | Installs a dependency system-wide, outside your project's build tree. | Used only for local sandbox testing (`apt install libeigen3-dev`) — not part of the shipped project, since it requires the user to have that package manager. |
| **git submodules** | Manually vendoring a dependency's source as a linked git repo inside yours. | No. |

`FetchContent` was chosen here specifically to satisfy "a stranger can `git clone`, build, and run tests following only the README" — no extra package manager to install first.

## Compilers

| Tool | Compiles | Notes |
|---|---|---|
| `g++` / `clang++` | Regular C++ (`.cpp`) | Your default compiler; used for everything except `CudaBackend.cpp`. |
| `nvcc` | CUDA (`.cu`, or `.cpp` files marked `LANGUAGE CUDA`) | Part of the CUDA Toolkit, installed separately. Only invoked when `FEM_ENABLE_CUDA=ON`; `enable_language(CUDA)` is what makes CMake go looking for it. |

## Typical build workflow

```bash
cmake -S . -B build -G Ninja          # configure: reads all CMakeLists.txt, generates build files
cmake --build build                    # build: actually compiles/links everything
ctest --test-dir build --output-on-failure   # run all registered tests
```
- `-S .` — source directory (where the root `CMakeLists.txt` lives).
- `-B build` — where to put generated build files and compiled output (kept out of the source tree, hence `build/` in `.gitignore`).
- `-G Ninja` — which build-file *generator* to use (Ninja is fast; `Unix Makefiles` and Visual Studio project generators are common alternatives).
- Toggle build options at configure time: `cmake -S . -B build -DFEM_BUILD_TESTS=OFF -DFEM_ENABLE_CUDA=ON`.

## How this project's build actually flows

1. Root `CMakeLists.txt` runs first: sets global options, fetches Eigen via `FetchContent`.
2. `add_subdirectory(src)` compiles `fem_core` **once**.
3. `add_subdirectory(apps)` builds `fem_demo`, linking the already-compiled `fem_core` (no recompilation).
4. `add_subdirectory(tests)` fetches GoogleTest, builds `fem_tests`, links `fem_core` again — same compiled `.a`, reused.
5. `add_subdirectory(benchmarks)` does the same for `benchmark_assembly`.

`fem_core` is compiled exactly once regardless of how many executables link it — that's the whole point of pulling it into its own library target instead of building the solver code separately into each executable.
