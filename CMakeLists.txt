# Root build file: fetches dependencies, builds the fem_core library,
# the demo app, unit tests, and benchmarks.
cmake_minimum_required(VERSION 3.20)
project(nonlinear_fem_solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(FEM_BUILD_TESTS "Build unit tests" ON)
option(FEM_BUILD_BENCHMARKS "Build benchmarks" ON)
option(FEM_ENABLE_CUDA "Build CudaBackend (Phase 3)" OFF)

if(FEM_ENABLE_CUDA)
  enable_language(CUDA)
endif()

include(FetchContent)

# Eigen: header-only, no build step needed.
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

# nlohmann/json: header-only, used to parse the simulation input file
# (see examples/example_problem.json and fem::io::loadSimulationConfig).
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

add_subdirectory(src)
add_subdirectory(apps)

if(FEM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(FEM_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

# Optional `docs` target: regenerates the Doxygen HTML docs on demand
# (`cmake --build build --target docs`). Not part of the default `all`
# target — doxygen isn't a hard build dependency — and not tied to file
# timestamps: a plain add_custom_target() with no OUTPUT re-runs doxygen
# unconditionally every time the target is invoked, so `docs/html` is
# always regenerated fresh from the current sources rather than going stale.
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
  add_custom_target(docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating Doxygen documentation into docs/html"
    VERBATIM
  )
else()
  message(STATUS "Doxygen not found — 'docs' target unavailable (install doxygen to enable it)")
endif()

# Copy example input files next to the build output so `fem_demo` can be
# run directly from the build directory without hunting for paths.
file(COPY ${CMAKE_SOURCE_DIR}/examples DESTINATION ${CMAKE_BINARY_DIR})
