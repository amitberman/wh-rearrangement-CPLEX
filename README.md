# wh-rearrangement# MAWR Build Guide (Flow Backend Selection)

This project supports two flow backends:

- OR-Tools (baseline)
- CPLEX (replacement solver for benchmarking)

Backend selection is **compile-time only**.

Each executable contains exactly one backend.

---

# Build OR-Tools Version

cmake -S . -B build_ortools \
  -DCMAKE_BUILD_TYPE=Release \
  -DFLOW_BACKEND=ORTOOLS \
  -DCMAKE_PREFIX_PATH="$HOME/or-tools_x86_64_rockylinux-9_cpp_v9.12.4544;$HOME/or-tools_x86_64_rockylinux-9_cpp_v9.12.4544/lib64/cmake"

cmake --build build_ortools -j

Verify:

grep FLOW_BACKEND build_ortools/CMakeFiles/MAWR.dir/flags.make

Expected:

-DFLOW_BACKEND_ORTOOLS

---

# Build CPLEX Version

cmake -S . -B build_cplex \
  -DCMAKE_BUILD_TYPE=Release \
  -DFLOW_BACKEND=CPLEX \
  -DCPLEX_ROOT=$HOME/cplex2212

cmake --build build_cplex -j

Verify:

grep FLOW_BACKEND build_cplex/CMakeFiles/MAWR.dir/flags.make

Expected:

-DFLOW_BACKEND_CPLEX

Note:
CPLEX is statically linked → it will not appear in ldd output.

---

# Run Solver Example

./build_ortools/MAWR -m <map> -s <scenario> -a NATCBS -t 60 -o results.csv --v 2

or

./build_cplex/MAWR -m <map> -s <scenario> -a NATCBS -t 60 -o results.csv --v 2

---

# Benchmark Workflow

Recommended comparison:

run OR-Tools version
run CPLEX version
compare runtime + solution quality

Keep both builds:

build_ortools/
build_cplex/

Use identical inputs for fair benchmarking.

### OR-Tools vs CPLEX Mode Benchmark (`scripts/run_bm.sh`)

`run_bm.sh` compares OR-Tools (`build_ortools/MAWR`) against CPLEX (`build_cplex/MAWR`) and writes `benchmark_results.md`.

Usage:

```bash
bash scripts/run_bm.sh <root_directory> [cplex_mode]
```

Examples:

```bash
# Default mode (same as cplex-warm)
bash scripts/run_bm.sh wh

# Explicit warm-start mode
bash scripts/run_bm.sh wh cplex-warm

# Model reuse only (no basis warm-start)
bash scripts/run_bm.sh wh cplex-reuse-only

# Basis warm-start only (no model reuse)
bash scripts/run_bm.sh wh cplex-basis-only

# Full cold-start baseline
bash scripts/run_bm.sh wh cplex-cold
```

Available `cplex_mode` values:

- `cplex-warm` -> `MAWR_CPLEX_WARM_START=1 MAWR_CPLEX_REUSE_MODEL=1`
- `cplex-reuse-only` -> `MAWR_CPLEX_WARM_START=0 MAWR_CPLEX_REUSE_MODEL=1`
- `cplex-basis-only` -> `MAWR_CPLEX_WARM_START=1 MAWR_CPLEX_REUSE_MODEL=0`
- `cplex-cold` -> `MAWR_CPLEX_WARM_START=0 MAWR_CPLEX_REUSE_MODEL=0`

The result table header is generated dynamically and reflects the selected CPLEX model label.

---

# Min-Flow Formulation & Full-Topology Mode

## Summary of Changes

Two major optimizations were implemented to improve CPLEX warm-start performance:

### 1. **Min-Flow Formulation Implementation**

- **File Modified:** `include/NATCBS/f_agents_planner_cplex.cpp` (lines ~100-127)
- **Change:** Replaced the augmented-network helper-arc abstraction with direct lower bounds on move-gadget arcs
- **Mathematical Basis:** Implements the min-flow formulation from the research paper, where positive edges are enforced via arc lower bounds instead of auxiliary helper nodes
- **Benefit:** Cleaner network structure and less topological changes
- **Validation:** 100% solution quality match against OR-Tools baseline (55 test scenarios, zero cost/makespan mismatches)
- **Note:** This change alone provides minimal rebuild reduction (~0.8%) because rebuilds are driven by network topology growth, not constraints

### 2. **Full-Topology Mode with Capacity-Zero**

- **Environment Variable:** `MAWR_FULL_TOPOLOGY_MODE` (default: 0)
- **Files Modified:**
  - `include/NATCBS/f_agents_planner.hpp`: Added full-topology members and state variables
  - `include/NATCBS/f_agents_planner_common.cpp`: Dual-path graph construction logic
- **How It Works:**
  1. At initialization, pre-computes reachability distances from all agent starts via BFS
  2. Pre-builds all passable map locations as nodes upfront
  3. Creates all candidate arcs, but sets capacity to 0 for arcs leading to locations unreachable by the required timestep
  4. Time layers continue to grow dynamically (as planning horizon expands)
- **Benefit:** Reduces rebuilds significantly
  - **Before:** ~630 rebuilds per 55-scenario benchmark (topology grows as reachable set expands)
  - **After:** ~55 rebuilds per 55-scenario benchmark (91% reduction, only initial build + a few topology completions)
- **Trade-off:** Slightly higher memory overhead for arc capacity storage, but stable topology enables efficient warm-start basis reuse across iterations
---

## Benchmark Results

### Full Warm-Start vs Full Cold (with Full-Topology Mode)

```
Configuration: MAWR_FULL_TOPOLOGY_MODE=1
Warm-Start:   MAWR_CPLEX_WARM_START=1 MAWR_CPLEX_REUSE_MODEL=1
Cold-Start:   MAWR_CPLEX_WARM_START=0 MAWR_CPLEX_REUSE_MODEL=0 (cold_mode=full)
Test Set:     55 benchmark scenarios

Results (warmstart_results_fullwarm_vs_fullcold.csv):
  Warm Total Time:     14.876 sec
  Cold Total Time:     20.486 sec
  Speedup (Cold/Warm): 1.377x
  Mean Per-Scenario:   1.098x
  
  Warm Basis Loads:    509 (reused across iterations)
  Cold Basis Loads:    0 (never used)
  
  Warm Rebuilds:       55 (one per scenario + initial)
  Cold Rebuilds:       639 (nearly every iteration)
  
  Warm Differential Updates: 586
  Cold Differential Updates: 0
```

**Interpretation:**
- Warm-start provides consistent **1.1–1.4x speedup** over full cold-start when network topology uses the same network.
- The modest speedup (vs naïve expectation of 2–3x) is because full-topology mode already makes cold-start fast

### OR-Tools vs CPLEX (with Full-Topology Mode)

```bash
bash scripts/run_bm.sh wh cplex-warm   # with MAWR_FULL_TOPOLOGY_MODE=1
```

Result: 55 scenarios, 100% cost/makespan match, CPLEX **6.986x faster** than OR-Tools.

---

# Warm-Start (CPLEX NATCBS Iterations)

Warm-start is implemented in the CPLEX flow backend for NATCBS iterative solves.

What is persisted across NATCBS flow solves:

- CPLEX environment (`CPXENVptr`)
- CPLEX network model (`CPXNETptr`)
- Network simplex basis (`CPXNETgetbase` / `CPXNETcopybase`)

Differential update policy (when network topology is unchanged):

- Arc capacity changes are applied via bounds update (`CPXNETchgbds`), including "remove arc" behavior with upper bound = `0`.
- Arc weight changes are applied via objective updates (`CPXNETchgobj`).

Fallback behavior:

- If network topology changes (node/arc incidence differs), the model is rebuilt via `CPXNETcopynet` and warm-start basis is reset.

---

## Runtime Toggles: `MAWR_CPLEX_WARM_START` vs `MAWR_CPLEX_REUSE_MODEL`

These two environment variables control different aspects of warm-start and can be combined:

### `MAWR_CPLEX_WARM_START` (default: 1)

**When enabled (=1):**
- After each successful min-cost flow solve, extract and save the optimal simplex basis (arc and node statuses)
- On the next iteration, if topology is unchanged, **inject the saved basis** via `CPXNETcopybase()` before solving
- Simplex algorithm uses this basis as a starting point → faster convergence (warm-start behavior)
- Output signal: `[CPLEX] Warm start basis loaded.`

**When disabled (=0):**
- Basis is never saved or reused
- Simplex always starts from scratch (cold-start behavior)
- Network model can still be reused (faster than cold-start single-solve)
- Output signal: `[CPLEX] No reusable basis available; cold start.`

**Performance impact:** Basis warm-start typically provides ~1-10% speedup depending on how much the flow network structure changes between iterations.

---

### `MAWR_CPLEX_REUSE_MODEL` (default: 1)

**When enabled (=1):**
- First iteration: create CPLEX environment and network model via `CPXNETcreateprob()`
- Subsequent iterations:
  - If topology unchanged: apply fast **differential updates** via `CPXNETchgbds()` (bounds) and `CPXNETchgobj()` (costs)
  - If topology changed: rebuild entire model via `CPXNETcopynet()`
- Avoids repeated model creation overhead
- Output signal: `[CPLEX] Applied differential update (bounds/objective).` or `[CPLEX] Rebuilt network model (topology changed).`

**When disabled (=0):**
- Model is discarded and rebuilt via `CPXNETcopynet()` on every iteration
- Reverts to original behavior: stateless per-iteration solves
- Higher overhead due to repeated allocation/deallocation
- Basis warm-start is ineffective (basis from one ephemeral model doesn't transfer to next)

## Usage Examples

The default configuration enables both warm-start basis and model reuse for maximum performance:

```bash
# Full warm-start (recommended for NATCBS iterations)
MAWR_CPLEX_WARM_START=1 MAWR_CPLEX_REUSE_MODEL=1 ./build_cplex/MAWR -m <map> -s <scenario> -a NATCBS -t 60 -o results.csv --v 2
```

Other configurations for comparison:

```bash
# Model reuse only (no basis reuse)
MAWR_CPLEX_WARM_START=0 MAWR_CPLEX_REUSE_MODEL=1 ./build_cplex/MAWR -m <map> -s <scenario> -a NATCBS -t 60 -o results.csv --v 2

# Basis warm-start only (model rebuilt each iteration)
MAWR_CPLEX_WARM_START=1 MAWR_CPLEX_REUSE_MODEL=0 ./build_cplex/MAWR -m <map> -s <scenario> -a NATCBS -t 60 -o results.csv --v 2

# Full cold-start baseline (for comparison)
MAWR_CPLEX_WARM_START=0 MAWR_CPLEX_REUSE_MODEL=0 ./build_cplex/MAWR -m <map> -s <scenario> -a NATCBS -t 60 -o results.csv --v 2
```

Debug signals in verbose mode (`--v 2`):

- `[CPLEX] Warm start basis loaded.` — Basis successfully injected before solve
- `[CPLEX] Applied differential update (bounds/objective).` — Bounds/costs updated without rebuild
- `[CPLEX] Rebuilt network model (topology changed).` — Network structure changed; full model rebuild occurred

---

# Warm-Start Performance Evaluation

## Build Steps (Warm-Start Benchmark Setup)

Build OR-Tools binary:

```bash
cmake -S . -B build_ortools \
  -DCMAKE_BUILD_TYPE=Release \
  -DFLOW_BACKEND=ORTOOLS \
  -DCMAKE_PREFIX_PATH="$HOME/or-tools_x86_64_rockylinux-9_cpp_v9.12.4544;$HOME/or-tools_x86_64_rockylinux-9_cpp_v9.12.4544/lib64/cmake"

cmake --build build_ortools -j
```

Build CPLEX binary:

```bash
cmake -S . -B build_cplex \
  -DCMAKE_BUILD_TYPE=Release \
  -DFLOW_BACKEND=CPLEX \
  -DCPLEX_ROOT=$HOME/cplex2212

cmake --build build_cplex -j
```

## Run Benchmarks

### OR-Tools vs CPLEX Mode Benchmark

Compare OR-Tools against CPLEX under various warm-start configurations:

```bash
# Default: CPLEX with full warm-start + full-topology mode
MAWR_FULL_TOPOLOGY_MODE=1 bash scripts/run_bm.sh wh cplex-warm
```

Other configurations:

```bash
# CPLEX with model reuse only (no basis warm-start)
MAWR_FULL_TOPOLOGY_MODE=1 bash scripts/run_bm.sh wh cplex-reuse-only

# CPLEX with basis warm-start only (model rebuilt each iteration)
MAWR_FULL_TOPOLOGY_MODE=1 bash scripts/run_bm.sh wh cplex-basis-only

# CPLEX full cold-start baseline (for comparison)
MAWR_FULL_TOPOLOGY_MODE=1 bash scripts/run_bm.sh wh cplex-cold

# Legacy mode (dynamic topology, original arc-omission strategy)
bash scripts/run_bm.sh wh cplex-warm  # MAWR_FULL_TOPOLOGY_MODE=0 (default)
```

**Output:** `benchmark_results.md` with per-scenario cost, makespan, and runtime comparison.

### Warm-Start vs Cold-Start Benchmark (CPLEX Only)

Run warm-start vs cold-start in A/B mode with CSV output:

`reuse`: Cold-start with model reuse but no basis warm-start (CPLEX can rebuild the model incrementally)
`full`: Truly cold—model destroyed and rebuilt every iteration, no basis injection
**Usage:** `bash scripts/benchmark_warmstart.sh <dir> [timeout] [max_scenarios] [output.csv] [cold_mode]`

```bash
bash scripts/benchmark_warmstart.sh <root_dir> [timeout_sec] [max_scenarios] [output_csv] [cold_mode]
```

| Column | Meaning |
|--------|---------|
| `warm_time_sec` | Warm-start solver time (with basis injection) |
| `cold_time_sec` | Cold-start solver time (specified by `cold_mode`) |
| `cold_over_warm` | Speedup ratio (e.g., 1.377x = cold took 1.377x longer) |
| `warm_basis_loads` | Number of times basis was successfully reused |
| `warm_diff_updates` | Number of differential updates (bounds/objective only) |
| `warm_rebuilds` | Number of full topology rebuilds in warm pipeline |
| `warm_rebuild_ratio` | Rebuild frequency: `warm_rebuilds / total_iterations` |
| `cold_diff_updates` | Diff updates in cold pipeline (0 for `full` mode) |
| `cold_rebuilds` | Rebuilds in cold pipeline (usually equals total scenarios) |

**Interpretation Guide:**

- **High `cold_over_warm`** (>1.2x): Warm-start is providing significant benefit
- **High `warm_basis_loads`**: Basis was reused frequently (topology stable)
- **Low `warm_rebuild_ratio`** (<0.1): Most iterations used differential updates (ideal)
- **High `warm_diff_updates`** relative to `warm_rebuilds`: Network mostly changed in arc weights/capacities, not structure

---

## Quick Benchmark Comparisons

### Scenario 1: Validate Correctness (OR-Tools Baseline)

```bash
# Build both backends
cmake -S . -B build_ortools ...
cmake -S . -B build_cplex ...

# Benchmark with full-topology mode
MAWR_FULL_TOPOLOGY_MODE=1 bash scripts/run_bm.sh wh cplex-warm

# Expected: Perfect cost and makespan match
```

### Scenario 2: Measure Warm-Start Benefit

```bash
# Warm-start vs cold-start (full rebuild baseline)
MAWR_FULL_TOPOLOGY_MODE=1 bash scripts/benchmark_warmstart.sh wh 60 0 results.csv full

# Should see cold_over_warm >= 1.1x for most scenarios
# Larger problems typically show 1.2–1.5x benefit
```

### Scenario 3: Compare Old vs New Topology Strategy

```bash
# Legacy (dynamic topology, arc omission)
bash scripts/benchmark_warmstart.sh wh 60 0 results_old.csv full

# New (full-topology, capacity-zero)
MAWR_FULL_TOPOLOGY_MODE=1 bash scripts/benchmark_warmstart.sh wh 60 0 results_new.csv full

# New should show:
# - Fewer rebuilds (91% reduction in warm_rebuilds + cold_rebuilds)
# - Lower overall times

