# CI/CD Plan for the Nonlinear FEM Solver

> **What is this document?** A step-by-step plan for adding automated
> building and testing ("CI/CD") to this project. Written so it can be
> understood a year from now without remembering any jargon.
> Words marked **in bold** are explained in the Glossary at the bottom.
>
> Date written: 2026-10-03

---

## 1. What are we building, in plain words?

Every time you push code to GitHub or open a pull request, GitHub will
**automatically**:

1. Download the code onto a fresh, empty virtual computer.
2. Build the project from scratch (proving it builds on a clean machine,
   not just on your laptop).
3. Run the whole test suite (proving the solver still gives correct answers).
4. Run extra safety checks (memory-error detection, code-quality analysis,
   code-coverage measurement).
5. Build the documentation website and publish it.
6. Run the benchmark program.
7. When you tag a version (like `v1.0.0`), package the program into a
   downloadable file attached to a GitHub "Release" page.

If anything fails, GitHub shows a red X and (once branch protection is
enabled) blocks merging the broken code into `main`.

**Why bother?** Today you only find out something is broken when you
remember to run the tests by hand, on your own machine, with your own
settings. CI runs the checks every time, on clean machines, on three
operating systems, and keeps the history.

---

## 2. Facts about this repo the plan relies on

- Build system: **CMake** (version 3.20 or newer) with the **Ninja**
  generator; C++17. Project name: `nonlinear_fem_solver`.
- External libraries (Eigen, nlohmann_json, GoogleTest) are downloaded
  automatically by CMake during the build ("FetchContent"). CI machines
  therefore need internet access — GitHub's machines have it.
- Things that get built:
  - `fem_core` — the solver library itself.
  - `fem_tests` — the test program (GoogleTest); runnable through **ctest**.
  - `fem_demo` — the command-line demo app; needs a `.json` input file
    from `examples/`.
  - `benchmark_assembly` — the benchmark program.
    ⚠️ It is currently an **empty skeleton**: it just prints
    "not yet implemented" and exits. This matters for Phase 6 below.
  - `docs` — the Doxygen documentation website (optional build target).
- There is a `FEM_ENABLE_CUDA` option but the CUDA backend is an empty
  placeholder (a **stub**) — so the plan does not test CUDA at all.
- Tests read input files from `examples/`; CMake copies that folder into
  the build directory automatically, so `ctest --test-dir build` just works.
- The small scripts in `build/` (`debug.sh`, `release.sh`, `check.sh`)
  are local conveniences only; CI calls CMake/ctest directly.
- Known local annoyance that CI avoids automatically: after heavily editing
  a test file, the saved list of tests on your machine can go **stale**
  (outdated) so `ctest` runs ghost tests. CI always builds in an empty
  folder, so this can never happen there.

---

## 3. Files that will be created

| File | What it is |
|---|---|
| `.github/workflows/ci.yml` | The main CI workflow: 6 jobs (below). **New file.** |
| `.github/workflows/release.yml` | The release/packaging workflow, runs on version tags. **New file.** |
| `README.md` | One line added: the CI status **badge** picture. |

A **workflow file** is a YAML text file that tells GitHub Actions what to
do: when to run, on what machines, and which commands to execute. No other
project files need to change — the build system already supports everything.

---

## 4. Step-by-step to-do list

The phases are ordered. Phase 1 must come first (the other jobs copy its
structure). Phases 2–6 can be done in any order after Phase 1, even in
parallel. Phase 7 is independent. Phase 8 is manual clicking in GitHub's
web interface — do it last, once the workflows exist.

### Phase 1 — Core workflow: build + test on 3 operating systems

- [x] **1.1** Create `.github/workflows/ci.yml` with:
  - **Triggers:** every push to `main` + every pull request targeting `main`.
  - **Concurrency rule:** if you push again while a run is still going,
    cancel the outdated run (saves time and runner minutes).
- [x] **1.2** Add job `build-test` with a **matrix** (the same steps repeated
  with different settings):
  - Operating systems: Ubuntu (Linux), macOS, Windows.
  - Build types: **Debug** and **Release**.
  - On Ubuntu, build once with **gcc** and once with **clang**
    (two different C++ compilers; each catches mistakes the other misses).
  - Result: about 5–6 parallel "legs" per run.
- [x] **1.3** Steps inside each leg:
  1. Check out the code.
  2. **Cache** the downloaded dependencies (the `build/_deps` folder),
     keyed on the operating system + a hash of the `CMakeLists.txt` files.
     This avoids re-downloading Eigen/GoogleTest/json on every single run.
  3. Configure: `cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=<Debug|Release>`
  4. Build: `cmake --build build -j`
  5. Test: `ctest --test-dir build --output-on-failure`
  6. **Smoke run** (quick "does it start and not crash" check):
     `./build/apps/fem_demo build/examples/analytical_newton.json`
     — this example has a mathematically known correct answer, so it is a
     trustworthy input.
- [x] **1.4** Local dry-run BEFORE pushing: in a brand-new empty build
  folder, run the exact configure/build/test commands above for Debug and
  Release and confirm everything passes. CI always builds from scratch, so
  you must verify a from-scratch build works.
- [x] **1.5** Push on a branch, open a PR, watch all legs go green in the
  "Actions" tab on GitHub. The PR itself is the end-to-end test.

### Phase 2 — Sanitizer job (memory-safety checking)

- [x] **2.1** Add job `sanitizers` to `ci.yml`: Ubuntu, gcc, Debug, with
  extra compiler flags `-fsanitize=address,undefined -fno-omit-frame-pointer`
  (both compile and link flags).
- [x] **2.2** Run the full test suite under these flags.
- [ ] **2.3** If it reports problems: each report is a real bug (memory
  corruption or **undefined behavior**). Fix the code — do not just silence
  the checker. If a report comes from inside the Eigen library itself, a
  targeted suppression is acceptable; document it in a comment.

*Why this job matters: the normal tests verify the solver computes correct
numbers. The sanitizer job verifies the program is not corrupting memory
while computing them. In C++, both kinds of bugs are common and the second
kind often passes normal tests silently.*

### Phase 3 — Code coverage job (which lines the tests exercise)

- [x] **3.1** Add job `coverage`: Ubuntu, gcc, Debug, compiled with
  `--coverage` flags.
- [x] **3.2** Run the tests, then generate a coverage report with `lcov`,
  excluding the downloaded dependencies (`_deps/`) and the test files
  themselves.
- [ ] **3.3** Upload the report to **Codecov** (a free website that draws
  nice coverage graphs and comments them on PRs). This needs:
  a free Codecov account, and a secret named `CODECOV_TOKEN` stored in the
  GitHub repo settings (Settings → Secrets and variables → Actions).
- [x] **3.4** Fallback if you don't want a Codecov account: attach the HTML
  report to the workflow run as a downloadable **artifact** instead.
- [ ] *No minimum-coverage enforcement at first — just measure and report.*

### Phase 4 — Lint job (automatic code-quality review)

- [x] **4.1** Add job `lint`: Ubuntu, install `clang-tidy`, run it over the
  project's own source files (exclude `_deps/`) using the
  `compile_commands.json` file CMake already generates.
- [x] **4.2** ⚠️ The first run will report MANY pre-existing warnings.
  Configure the job as **report-only** at first (it posts results but does
  not fail the run), otherwise every PR would look broken from day one.
- [ ] **4.3** Later, as a separate cleanup task: fix the backlog of
  warnings, then flip the job to fail on new warnings.

### Phase 5 — Documentation job + publishing to GitHub Pages

- [x] **5.1** Add job `docs`: Ubuntu, install Doxygen, build the `docs`
  target (`cmake --build build --target docs`).
- [ ] **5.2** Check the `WARN_AS_ERROR` setting in `Doxyfile`; if feasible,
  make documentation warnings fail the job so broken doc comments get
  caught.
- [x] **5.3** On pushes to `main` only, publish the generated HTML
  (`docs/html`) to **GitHub Pages** (a free project website hosted by
  GitHub) using GitHub's official Pages actions.
  One-time manual setup: repo Settings → Pages → set source to
  "GitHub Actions".

### Phase 6 — Benchmark job

- [x] **6.1** Add job `benchmark`: Ubuntu, Release build, run
  `./build/benchmarks/benchmark_assembly`, save its output as an artifact.
- [ ] **6.2** ⚠️ **BLOCKED for real use:** `benchmark_assembly` is currently
  an empty skeleton (it prints a message and exits 0). Today this job only
  proves the program starts. **Performance tracking requires implementing
  the benchmark first** — this is "Step 1.3" in the project's own plan
  document (`docs/Cheatsheets/nonlinear_FEM_solver_project_plan_v2.md`).
- [ ] **6.3** Once the benchmark is implemented (separate task): make it
  print results as JSON, then add the `github-action-benchmark` action,
  which stores results over time and fails a PR if performance regresses
  beyond a chosen threshold.

### Phase 7 — Release & packaging workflow

- [x] **7.1** Create `.github/workflows/release.yml`, triggered by pushing
  a **tag** matching `v*` (e.g. `git tag v1.0.0 && git push --tags`).
- [x] **7.2** Build in Release mode; package `fem_demo`,
  `benchmark_assembly`, and the `examples/` folder into an archive:
  `.tar.gz` for Linux, `.zip` for Windows.
- [x] **7.3** Create a GitHub "Release" page for the tag and attach the
  archives automatically (`softprops/action-gh-release` does this).
- [ ] **7.4** Verify with a throwaway tag like `v0.0.1-rc1`; delete the
  test release afterwards.

### Phase 8 — Badge + branch protection (manual, GitHub website)

- [x] **8.1** Add the status **badge** to the top of `README.md`. Easiest
  way: GitHub → Actions tab → click the "CI" workflow → "···" menu →
  "Create status badge" → copy the markdown line → paste into README.md.
  It looks like:
  `![CI](https://github.com/<you>/<repo>/actions/workflows/ci.yml/badge.svg)`
  Add the Codecov badge too once Phase 3 works.
- [ ] **8.2** Enable **branch protection** on `main`
  (Settings → Branches → Add rule): require the `build-test`,
  `sanitizers`, and `coverage` checks to pass before merging, and require
  branches to be up to date.

---

## 5. How to verify it all works

1. **Local dry-run** (Phase 1.4) — exact CI commands, fresh folder, green.
2. **The first PR** — the pull request that adds `ci.yml` is itself the
   end-to-end test; every job must be green in the Actions tab.
3. **Deliberate breakage** — push a throwaway commit that breaks a test;
   confirm the pipeline turns red; revert the commit.
4. **Test release** — push tag `v0.0.1-rc1`, confirm a Release page with
   downloadable archives appears; delete it afterwards.
5. **Docs site** — after the first `main` push, open the GitHub Pages URL
   and confirm the documentation renders.
6. **Coverage** — open the Codecov report (or download the artifact) and
   confirm real numbers appear.

---

## 6. Decisions made (and why)

- **Platform: GitHub Actions.** The repo already lives on GitHub
  (`.github/` exists); free runners for Linux/macOS/Windows.
- **Runs on:** push to `main` + pull requests. No nightly schedule for now.
- **Everything included:** macOS/Windows builds, coverage, releases, docs,
  benchmark, lint — all requested explicitly by the owner.
- **No CUDA job:** the GPU backend is an empty placeholder; nothing to test
  yet. Revisit when real GPU code lands.
- **Lint starts report-only:** otherwise old warnings block every PR.
- **Coverage is measured but not enforced** (no "must be above X%" gate yet).
- **Performance gating is planned but blocked** on implementing the
  benchmark skeleton (Phase 6.2).
- **CI calls cmake/ctest directly**, not the helper scripts in `build/`.

---

## 7. Known risks / things to watch

1. **PR slowness:** the full matrix can take 10–20 minutes. If that gets
   annoying: trim macOS/Windows legs to Release-only, or move
   coverage/sanitizers to a nightly schedule.
2. **Codecov account:** Phase 3 needs an external account + secret. The
   artifact fallback avoids this entirely.
3. **Sanitizer findings:** expect ASan/UBSan to find real bugs on the first
   run — budget time to fix them; that's the point of the job.
4. **Windows leg:** commands were chosen to be portable (plain cmake/ctest),
   but small adjustments (paths, shell syntax) may be needed on first run.

---

## 8. Glossary (plain-English definitions)

### CI vs CD — the two halves of this plan

**CI = Continuous Integration.** *"Continuously verify that new code
integrates safely with the existing codebase."* Every change is
automatically built and tested, so problems surface within minutes instead
of weeks later. CI is everything about **verification**.

**CD = Continuous Delivery.** *"Continuously produce a packaged,
ready-to-use product."* After code passes verification, automatically turn
it into something distributable: downloadable binaries, a published
website. CD is everything about **delivering artifacts to users**.

Which phases are which:

| Phase | CI or CD | Why |
|---|---|---|
| 1: build + test matrix | **CI** | Verifies the code compiles and tests pass on 3 operating systems |
| 2: sanitizers | **CI** | Verifies memory safety |
| 3: coverage | **CI** | Verifies how much code the tests exercise |
| 4: lint | **CI** | Verifies code quality |
| 5a: *building* docs (catching doc errors) | **CI** | Verification |
| 5b: *publishing* docs to GitHub Pages | **CD** | Delivers the docs website to readers |
| 6: benchmark | **CI** | Verifies performance didn't regress |
| 7: release packaging + GitHub Release | **CD** | Turns a git tag into downloadable archives on a Release page |
| 8: badge + branch protection | neither | Repo configuration that *uses* CI results |

This plan is mostly CI with a CD tail — typical, because CD only makes
sense once CI proves the thing being delivered actually works.

### Glossary table

| Term | Meaning |
|---|---|
| **CI / CD** | See the full explanation above this table. |
| **Workflow** | A YAML file in `.github/workflows/` describing automated steps GitHub runs for you. |
| **Job** | One block of a workflow; jobs run in parallel on separate virtual machines. |
| **Runner** | The virtual machine GitHub provides to execute a job. "ubuntu-latest" = a fresh Linux machine. |
| **Matrix** | Repeating the same job with different settings (e.g. 3 operating systems × 2 build types) in parallel. Each combination is a "leg". |
| **PR / pull request** | A proposal to merge code changes into a branch, reviewable on GitHub. |
| **Push** | Uploading your local commits to GitHub. |
| **Tag** | A named marker on a commit, used for versions like `v1.0.0`. |
| **CMake** | The tool that generates the build system for this project. |
| **Ninja** | The low-level tool that actually compiles the files (CMake drives it). |
| **Debug / Release** | Build flavors: Debug = slower binary with full error checking and debug info; Release = fast optimized binary. |
| **gcc / clang** | Two different C++ compilers. Building with both catches more mistakes. |
| **ctest** | The test runner that comes with CMake; it finds and runs the GoogleTest tests. |
| **Smoke run** | A quick "does it start without crashing?" check, named after powering up hardware and watching for smoke. |
| **Sanitizer / ASan / UBSan** | Special compiler modes that detect memory corruption (AddressSanitizer) and illegal/undefined program behavior (UndefinedBehaviorSanitizer) while the tests run. Normal tests check the *numbers* are right; sanitizers check the program isn't corrupting memory *while* computing them — and memory bugs often pass normal tests silently. |
| **Undefined behavior** | Code the C++ standard forbids (e.g. signed integer overflow); often *appears* to work, which is what makes it dangerous. |
| **Lint / clang-tidy** | Automatic code review without running the program: finds suspicious or bug-prone patterns. |
| **Coverage** | A measurement of *which lines of source code the tests actually executed*. Tests can all pass while large parts of the code are never exercised — bugs there surface later, in real use. A report shows per-file percentages plus line-by-line green (executed) / red (never executed) highlighting. What you need it for: (1) finding blind spots — red lines show exactly where to write new tests; (2) preventing silent decay — a PR adding 200 lines of solver code with no tests visibly drops the number; (3) giving meaning to "all tests pass" by telling you how much code those tests actually reach. For a correctness-critical numerics codebase this is one of the most valuable extra jobs. |
| **Codecov** | A free website (codecov.io) that hosts coverage reports. CI uploads raw coverage data there; Codecov draws trend graphs over time, comments on pull requests (e.g. "this PR changes coverage by −2.4%, uncovered lines: MeshBuilder.cpp 45–52"), and provides a README badge. Optional convenience: the plan has a fallback where CI just attaches the report as a downloadable artifact instead. Requires a free account + a `CODECOV_TOKEN` secret. |
| **Stub / skeleton** | A placeholder implementation that exists but does nothing real yet (e.g. `CudaBackend`, `benchmark_assembly`). |
| **Doxygen** | The tool that generates the HTML documentation website from code comments. |
| **GitHub Pages** | Free static-website hosting from GitHub; used here to publish the docs. |
| **Badge** | The small "build \| passing" image at the top of READMEs; updates automatically from CI status. |
| **Branch protection** | GitHub setting that forbids merging into `main` unless CI checks pass. |
| **Artifact** | A file a workflow run saves for download (e.g. the coverage report HTML). |
| **Cache** | A snapshot of files saved on GitHub's servers and restored into later runs to avoid repeating slow work. Here: after the first run downloads Eigen/GoogleTest/json into `build/_deps`, that folder is saved; later runs restore it instead of re-downloading (saves 1–2 minutes × ~6 jobs × every push). The snapshot is stored under a "key" containing a fingerprint (hash) of the `CMakeLists.txt` files — change a library version there and the fingerprint changes, the old snapshot is discarded, and fresh libraries are downloaded once and re-cached. Analogy: a stocked pantry instead of re-ordering the same groceries every week. |
| **Secret** | A password/token stored safely in repo settings so workflows can use it without exposing it. |
| **FetchContent** | CMake feature that downloads libraries (Eigen, GoogleTest, json) automatically during configure. |
| **Stale** | Outdated, left over from before (e.g. an old cached test list that no longer matches the code). |
| **Release** | A GitHub page for a version tag, with downloadable packaged binaries attached. |
| **analytical_newton.json** | Example input whose correct answer is known mathematically ("analytical"), ideal for the smoke run. |
