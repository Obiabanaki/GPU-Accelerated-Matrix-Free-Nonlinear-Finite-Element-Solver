# fem_core: the library everything else links against.
add_library(fem_core
    fem/material/NeoHookeanMaterial.cpp
    fem/material/MooneyRivlinMaterial.cpp
    fem/element/Hex8Element.cpp
    fem/element/Tet4Element.cpp
    fem/mesh/GlobalSystem.cpp
    fem/mesh/Mesh.cpp
    fem/mesh/MeshBuilder.cpp
    fem/bc/DirichletBC.cpp
    fem/bc/ContactBC.cpp
    fem/linalg/LinearOperator.cpp
    fem/linalg/EigenSparseOperator.cpp
    fem/linalg/BackendOperator.cpp
    fem/linalg/IdentityPreconditioner.cpp
    fem/linalg/JacobiPreconditioner.cpp
    fem/linalg/ILUPreconditioner.cpp
    fem/linalg/DirectSolver.cpp
    fem/linalg/ConjugateGradientSolver.cpp
    fem/linalg/GMRESSolver.cpp
    fem/backend/DeviceCsrMatrix.cpp
    fem/backend/CpuBackend.cpp
    fem/solver/NewtonSolver.cpp
    fem/factory/Factory.cpp
    fem/io/InputReader.cpp
)

if(FEM_ENABLE_CUDA)
    target_sources(fem_core PRIVATE fem/backend/CudaBackend.cpp)
    set_source_files_properties(fem/backend/CudaBackend.cpp PROPERTIES LANGUAGE CUDA)
endif()

target_include_directories(fem_core PUBLIC ${CMAKE_SOURCE_DIR}/include)
# PRIVATE: lets fem_core's own .cpp files reach internal helper headers
# (e.g. fem/internal/VoigtUtil.hpp) that live under src/, not include/.
# Deliberately not PUBLIC — fem_demo/fem_tests/benchmark_assembly must
# not be able to reach these; that's what makes them "internal."
target_include_directories(fem_core PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fem_core PUBLIC Eigen3::Eigen)
# PRIVATE: JSON parsing is an implementation detail of fem/io/InputReader.cpp
# only (SimulationConfig.hpp stays dependency-free) — nothing downstream
# needs to know nlohmann::json exists.
target_link_libraries(fem_core PRIVATE nlohmann_json::nlohmann_json)
target_compile_features(fem_core PUBLIC cxx_std_17)
