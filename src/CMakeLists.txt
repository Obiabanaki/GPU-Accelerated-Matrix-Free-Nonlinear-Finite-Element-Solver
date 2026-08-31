# fem_core: the library everything else links against.
add_library(fem_core
    fem/material/NeoHookeanMaterial.cpp
    fem/material/MooneyRivlinMaterial.cpp
    fem/element/Hex8Element.cpp
    fem/element/Tet4Element.cpp
    fem/mesh/GlobalSystem.cpp
    fem/mesh/Mesh.cpp
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
)

if(FEM_ENABLE_CUDA)
    target_sources(fem_core PRIVATE fem/backend/CudaBackend.cpp)
    set_source_files_properties(fem/backend/CudaBackend.cpp PROPERTIES LANGUAGE CUDA)
endif()

target_include_directories(fem_core PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(fem_core PUBLIC Eigen3::Eigen)
target_compile_features(fem_core PUBLIC cxx_std_17)
