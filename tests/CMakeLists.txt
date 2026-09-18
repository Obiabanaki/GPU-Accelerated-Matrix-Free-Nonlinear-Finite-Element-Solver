include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.2
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(fem_tests
    test_material.cpp
    test_element.cpp
    test_linear_operator.cpp
    test_newton_solver.cpp
    test_pipeline.cpp
)
target_link_libraries(fem_tests PRIVATE fem_core GTest::gtest_main)
# Lets test_pipeline.cpp find examples/example_problem.json regardless of
# ctest's working directory (which is the test binary's own build dir, not
# the source root).
target_compile_definitions(fem_tests PRIVATE FEM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(fem_tests)
