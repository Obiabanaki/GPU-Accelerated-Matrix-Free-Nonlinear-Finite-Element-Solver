---
name: fem-cpp-conventions
description: 'Coding conventions for this GPU-Accelerated Matrix-Free Nonlinear FEM Solver repo. Use when writing or editing C++ code under include/fem, src/fem, tests, benchmarks, or apps — element formulations (Hex8Element, Element interface), materials, backends, boundary conditions, doc-comment style, and Eigen usage patterns.'
---

# FEM Solver C++ Conventions

## When to Use
-any change in the C++ code 

## File Layout
- Headers in `include/fem/<module>/Name.hpp`, implementation in `src/fem/<module>/Name.cpp`. Header has `#pragma once` and only declarations; `.cpp` includes the matching header first.
- Everything lives in `namespace fem { ... }` (anonymous `namespace { ... }` inside `.cpp` for file-local helpers like `strainDisplacementMatrix`, `toVoigt`).
- One class per header, named after the file (e.g. `Hex8Element.hpp` → `class Hex8Element`).

## Doc Comments
- File header: `/// @file Name.hpp` + `/// @brief <one line>`.
- Class/function docs: `/// @brief <one line>`, then `@param`/`@return` only when non-obvious.
- Overrides use `/// @copydoc Element::methodName` instead of repeating the description.
- Member variables get a trailing `///< short description` comment.
- Keep comments to what the code can't show itself — no restating the next line, no multi-paragraph doc blocks for simple functions.
- Non-trivial numerical formulations (e.g. total-Lagrangian residual/stiffness, Voigt ordering, shear factors) get a short block comment at the top of the `.cpp` file explaining the math convention once, not repeated inline.
- add comment for each non-trivial function, explaining its purpose, inputs, outputs, and any assumptions.
- the c++ programming tricks used should be documented with comments explaining their purpose and any assumptions or limitations.
- all loops should be commented with what is the loop over 

## Consistency
-It should be clarified that how each part of code is working with other part of the code. 
. in comments, explain what is input and output and where and how this part is going to be used.
- in comments, point out if there is any limitation in the use or assumptions made by this part of the code.

# unit tests
- for each class there should be corresponding unit tests covering typical usage, edge cases, and error conditions.
- for each numerical formulation (e.g., total-Lagrangian residual/stiffness, Voigt ordering), there should be unit tests verifying correctness against known solutions or finite-difference checks.
- unit tests should also cover the handling of edge cases such as degenerate elements, inverted elements, and boundary conditions.