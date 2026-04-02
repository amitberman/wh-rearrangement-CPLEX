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

