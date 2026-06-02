#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 4 ]]; then
    echo "Usage: $0 <root_directory> [timeout_sec=600] [max_scenarios=0] [out_md=ortools_vs_cplex_warm.md]"
    exit 1
fi

ROOT_DIR="$1"
TIMEOUT_SEC="${2:-600}"
MAX_SCENARIOS="${3:-0}"
OUT_MD="${4:-ortools_vs_cplex_warm.md}"

ORTOOLS_BIN="./build_ortools/MAWR"
CPLEX_BIN="./build_cplex/MAWR"

[[ -x "$ORTOOLS_BIN" ]] || { echo "Error: missing $ORTOOLS_BIN"; exit 1; }
[[ -x "$CPLEX_BIN" ]] || { echo "Error: missing $CPLEX_BIN"; exit 1; }
[[ -d "$ROOT_DIR" ]] || { echo "Error: directory '$ROOT_DIR' does not exist."; exit 1; }

mapfile -t SCEN_FILES < <(find "$ROOT_DIR" -type f -name "*.scen" | sort)

if [[ "$MAX_SCENARIOS" -gt 0 ]]; then
    SCEN_FILES=("${SCEN_FILES[@]:0:$MAX_SCENARIOS}")
fi

[[ ${#SCEN_FILES[@]} -gt 0 ]] || { echo "No scenarios found under '$ROOT_DIR'."; exit 1; }

TMP_OR_LOG="$(mktemp)"
TMP_CPLEX_LOG="$(mktemp)"
trap 'rm -f "$TMP_OR_LOG" "$TMP_CPLEX_LOG"' EXIT

extract_elapsed() {
    grep -E "Elapsed time:" "$1" | tail -n 1 | sed -E 's/.*Elapsed time: *([0-9]+(\.[0-9]+)?) sec.*/\1/' || echo "-"
}

extract_makespan() {
    grep -E "^Makespan:" "$1" | tail -n 1 | sed -E 's/Makespan: *([0-9]+).*/\1/' || echo "-"
}

calc_diff() {
    local or_time="$1"
    local cplex_time="$2"

    if [[ "$or_time" == "-" || "$cplex_time" == "-" ]]; then
        echo "-"
        return
    fi

    awk -v o="$or_time" -v c="$cplex_time" 'BEGIN {
        diff = o - c;
        ratio = (c > 0 ? o / c : 0);
        printf "%.3f sec, %.3fx", diff, ratio;
    }'
}

{
    echo "# OR-Tools vs CPLEX Warm-Start"
    echo
    echo "- Root directory: \`$ROOT_DIR\`"
    echo "- Timeout: ${TIMEOUT_SEC} sec"
    echo "- Total scenarios: ${#SCEN_FILES[@]}"
    echo "- CPLEX mode: \`MAWR_CPLEX_WARM_START=1 MAWR_CPLEX_REUSE_MODEL=1\`"
    echo
    echo "| Dataset | Scenario | Makespan Match | OR Makespan | CPLEX Makespan | OR Time (sec) | CPLEX Warm Time (sec) | Diff / Ratio |"
    echo "|---|---|---:|---:|---:|---:|---:|---:|"
} > "$OUT_MD"

i=0

for scen in "${SCEN_FILES[@]}"; do
    i=$((i + 1))

    scen_dir="$(dirname "$scen")"
    dataset="$(basename "$scen_dir")"

    echo "[$i/${#SCEN_FILES[@]}] Running $(basename "$scen")..."

    mapfile -t MAP_FILES < <(find "$scen_dir" -maxdepth 1 -name "*.map" | sort)

    if [[ ${#MAP_FILES[@]} -ne 1 ]]; then
        echo "Skipping '$dataset' / '$(basename "$scen")' (expected exactly one .map)."
        continue
    fi

    map_file="${MAP_FILES[0]}"

    "$ORTOOLS_BIN" \
        -m "$map_file" \
        -s "$scen" \
        -a NATCBS \
        -t "$TIMEOUT_SEC" \
        -o /tmp/mawr_ortools_eval.csv \
        --v 2 > "$TMP_OR_LOG" 2>&1 || true

    MAWR_CPLEX_WARM_START=1 MAWR_CPLEX_REUSE_MODEL=1 "$CPLEX_BIN" \
        -m "$map_file" \
        -s "$scen" \
        -a NATCBS \
        -t "$TIMEOUT_SEC" \
        -o /tmp/mawr_cplex_warm_eval.csv \
        --v 2 > "$TMP_CPLEX_LOG" 2>&1 || true

    or_time="$(extract_elapsed "$TMP_OR_LOG")"
    cplex_time="$(extract_elapsed "$TMP_CPLEX_LOG")"

    or_makespan="$(extract_makespan "$TMP_OR_LOG")"
    cplex_makespan="$(extract_makespan "$TMP_CPLEX_LOG")"

    if [[ "$or_makespan" == "$cplex_makespan" && "$or_makespan" != "-" ]]; then
        makespan_match="MATCH"
    else
        makespan_match="MISMATCH"
    fi

    diff_ratio="$(calc_diff "$or_time" "$cplex_time")"

    echo "| \`$dataset\` | \`$(basename "$scen")\` | $makespan_match | $or_makespan | $cplex_makespan | $or_time | $cplex_time | $diff_ratio |" >> "$OUT_MD"
done

echo "Benchmark complete: $OUT_MD"