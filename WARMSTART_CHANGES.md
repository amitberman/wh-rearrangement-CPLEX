# Warm-Start Benchmark Notes

## Purpose
This document summarizes the warm-start vs cold-start benchmarking changes and the measured aggregate result.

## What Was Added
- Benchmark script: `scripts/benchmark_warmstart.sh`
  - Runs each scenario twice with the same input paths:
    - Warm-start run: `MAWR_CPLEX_WARM_START=1 MAWR_CPLEX_REUSE_MODEL=1`
    - Cold-start run: `MAWR_CPLEX_WARM_START=0 MAWR_CPLEX_REUSE_MODEL=1`
  - Uses the same executable path for both runs: `./build_cplex/MAWR`
  - Writes CSV output to `warmstart_results.csv` (default)
- README updates in `README.md`
  - Added warm-start usage section
  - Added warm-start benchmark command and output column explanation

## Fixed Paths Used
- Solver binary: `./build_cplex/MAWR`
- Dataset root: `wh`
- Benchmark script: `scripts/benchmark_warmstart.sh`
- Output file: `warmstart_results.csv`

## Command Used
```bash
bash scripts/benchmark_warmstart.sh wh 60 0 warmstart_results.csv
```

## Metric Requested: Average Cold/Warm Ratio
Computed from column `cold_over_warm` in `warmstart_results.csv`:

- Number of scenarios: `55`
- Arithmetic mean of `cold_over_warm`: `1.015655x`

Interpretation:
- `cold_over_warm > 1` means warm-start is faster.
- Mean `1.015655x` indicates a small average warm-start advantage on this run.
