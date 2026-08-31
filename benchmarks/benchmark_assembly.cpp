/// @file benchmark_assembly.cpp
/// @brief Step 1.3 acceptance criterion: assembly runtime should scale
/// roughly linearly with element count. Also the harness Step 2.2's
/// preconditioner comparison and Step 3.4's CPU-vs-GPU SpMV comparison
/// extend, once those exist.
#include <chrono>
#include <iostream>

int main() {
    // TODO(Step 1.3): build structured cube meshes of increasing element
    // count, time Mesh::assemble for each, print/plot runtime vs. count
    // to confirm roughly linear scaling.
    std::cout << "benchmark_assembly: skeleton, not yet implemented.\n";
    return 0;
}
