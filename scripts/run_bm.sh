#!/usr/bin/env bash
set -u

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <scenario_directory>"
    exit 1
fi

SCEN_DIR="$1"
ORTOOLS_BIN="./build_ortools/MAWR"
CPLEX_BIN="./build_cplex/MAWR"
OUT_CSV="benchmark_results.csv"
TMP_LOG="$(mktemp)"

if [[ ! -d "$SCEN_DIR" ]]; then
    echo "Error: directory '$SCEN_DIR' does not exist."
    exit 1
fi

if [[ ! -x "$ORTOOLS_BIN" ]]; then
    echo "Error: OR-Tools binary not found or not executable: $ORTOOLS_BIN"
    exit 1
fi

if [[ ! -x "$CPLEX_BIN" ]]; then
    echo "Error: CPLEX binary not found or not executable: $CPLEX_BIN"
    exit 1
fi

mapfile -t MAP_FILES < <(find "$SCEN_DIR" -maxdepth 1 -type f -name "*.map" | sort)
if [[ ${#MAP_FILES[@]} -ne 1 ]]; then
    echo "Error: expected exactly one .map file in '$SCEN_DIR', found ${#MAP_FILES[@]}."
    exit 1
fi
MAP_FILE="${MAP_FILES[0]}"

mapfile -t SCEN_FILES < <(find "$SCEN_DIR" -maxdepth 1 -type f -name "*.scen" | sort)
if [[ ${#SCEN_FILES[@]} -eq 0 ]]; then
    echo "Error: no .scen files found in '$SCEN_DIR'."
    exit 1
fi

echo "backend,scenario,map,min_cost,makespan,elapsed_time,exit_code" > "$OUT_CSV"

run_one() {
    local backend="$1"
    local bin="$2"
    local scen="$3"

    "$bin" \
        -m "$MAP_FILE" \
        -s "$scen" \
        -a NATCBS \
        -t 60 \
        -o results.csv \
        --v 2 > "$TMP_LOG" 2>&1

    local exit_code=$?

    local min_cost=""
    local makespan=""
    local elapsed_time=""

    if grep -q "Minimum cost_type flow:" "$TMP_LOG"; then
        min_cost="$(grep "Minimum cost_type flow:" "$TMP_LOG" | tail -n 1 | sed 's/.*Minimum cost_type flow: *//')"
    fi

    if grep -q "^Makespan:" "$TMP_LOG"; then
        makespan="$(grep "^Makespan:" "$TMP_LOG" | tail -n 1 | sed 's/^Makespan: *//')"
    fi

    if grep -q "^Elapsed time:" "$TMP_LOG"; then
        elapsed_time="$(grep "^Elapsed time:" "$TMP_LOG" | tail -n 1 | sed 's/^Elapsed time: *//; s/ *sec$//')"
    fi

    echo "${backend},${scen},${MAP_FILE},${min_cost},${makespan},${elapsed_time},${exit_code}" >> "$OUT_CSV"
}

for scen in "${SCEN_FILES[@]}"; do
    run_one "ortools" "$ORTOOLS_BIN" "$scen"
    run_one "cplex" "$CPLEX_BIN" "$scen"
done

rm -f "$TMP_LOG"
echo "Done. Results written to $OUT_CSV"