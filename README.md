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

**What it controls:** Whether to reuse the previously computed **simplex basis** from the last iteration.

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

**What it controls:** Whether to reuse and incrementally update the persistent **network model** structure across iterations.

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

**Performance impact:** Model reuse provides ~10-30% speedup depending on solver overhead relative to problem size.

---

## Recommended Configurations

| Configuration | Warm-Start Basis? | Model Reuse? | Behavior | Use Case |
|--- |---|---|---|---|
| `=1, =1` (default) | ✓ | ✓ | **Full warm-start:** fast differential updates + basis injection | Iterative solving (NATCBS with changing topology) |
| `=1, =0` | ✓ | ✗ | Basis reused across ephemeral models (less effective) | Debugging; model rebuild cost not dominant |
| `=0, =1` | ✗ | ✓ | Model reused but cold-start each time | Topology changes frequently; basis less valuable |
| `=0, =0` | ✗ | ✗ | **Full cold-start:** rebuild + no basis reuse | Baseline comparison; one-off solves |

---

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

Run warm-start vs cold-start comparison on a sequence of scenarios:

bash scripts/benchmark_warmstart.sh wh

Example with explicit timeout, full scenario sweep, and custom output file:

bash scripts/benchmark_warmstart.sh wh 60 0 warmstart_results_after_patch.csv

Optional arguments:

bash scripts/benchmark_warmstart.sh <root_directory> [timeout_sec=60] [max_scenarios=0] [out_csv=warmstart_results.csv]

Output CSV columns (`warmstart_results.csv`):

- `warm_time_sec`, `cold_time_sec`, `cold_over_warm`
- `warm_basis_loads` (how often basis was reused)
- `warm_diff_updates`, `warm_rebuilds`
- `warm_rebuild_ratio` (proxy for magnitude of structural network changes)

Interpretation:

- Higher `cold_over_warm` means warm-start is helping.
- Lower `warm_rebuild_ratio` means more iterations used differential updates and are more likely to benefit from warm-start.

