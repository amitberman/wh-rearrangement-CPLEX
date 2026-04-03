#!/usr/bin/env bash
set -u

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <root_directory>"
    exit 1
fi

ROOT_DIR="$1"

ORTOOLS_BIN="./build_ortools/MAWR"
CPLEX_BIN="./build_cplex/MAWR"
CPLEX_STG3_BIN="./build_cplex_w_lb_3stage/MAWR"

OUT_MD="benchmark_results_or_cplex_stg3.md"
TMP_LOG="$(mktemp)"

if [[ ! -d "$ROOT_DIR" ]]; then
    echo "Error: directory '$ROOT_DIR' does not exist."
    exit 1
fi

for bin in "$ORTOOLS_BIN" "$CPLEX_BIN" "$CPLEX_STG3_BIN"; do
    if [[ ! -x "$bin" ]]; then
        echo "Error: binary not found or not executable: $bin"
        exit 1
    fi
done

mapfile -t SCEN_FILES < <(find "$ROOT_DIR" -type f -name "*.scen" | sort)

TOTAL_SCENS=${#SCEN_FILES[@]}
TOTAL_RUNS=$((TOTAL_SCENS * 3))
CURRENT_RUN=0

progress_bar() {
    local current="$1"
    local total="$2"
    local width=40

    local filled=$(( current * width / total ))
    local empty=$(( width - filled ))
    local percent=$(( current * 100 / total ))

    printf "\r["
    printf "%0.s#" $(seq 1 "$filled")
    printf "%0.s-" $(seq 1 "$empty")
    printf "] %3d%% (%d/%d)" "$percent" "$current" "$total"
}

run_one() {
    local bin="$1"
    local map_file="$2"
    local scen="$3"

    "$bin" \
        -m "$map_file" \
        -s "$scen" \
        -a NATCBS \
        -t 60 \
        -o results.csv \
        --v 2 > "$TMP_LOG" 2>&1

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

    echo "${min_cost}|${makespan}|${elapsed_time}"
}

calc_ratio() {
    local a="$1"
    local b="$2"

    if [[ "$a" == "-" || "$b" == "-" ]]; then
        echo "-"
        return
    fi

    awk -v x="$a" -v y="$b" 'BEGIN {
        if (y > 0) printf "%.2fx", x / y;
        else print "-";
    }'
}

match_status_3() {
    local a="$1"
    local b="$2"
    local c="$3"

    if [[ "$a" == "-" || "$b" == "-" || "$c" == "-" ]]; then
        echo "-"
    elif [[ "$a" == "$b" && "$b" == "$c" ]]; then
        echo "OK"
    else
        echo "MISMATCH"
    fi
}

{
    echo "# Benchmark Results"
    echo
    echo "- Root directory: \`$ROOT_DIR\`"
    echo "- Total scenarios: $TOTAL_SCENS"
    echo
    echo "| Dataset | Scenario | Cost Match | Makespan Match | Time (OR) | Time (CPLEX) | Time (CPLEX-STG3) | Speedup OR/CPLEX | Speedup OR/CPLEX-STG3 | Rate CPLEX/CPLEX-STG3 |"
    echo "|---|---|---|---|---:|---:|---:|---:|---:|---:|"
} > "$OUT_MD"

for scen in "${SCEN_FILES[@]}"; do
    scen_dir="$(dirname "$scen")"
    dataset="$(basename "$scen_dir")"

    mapfile -t MAP_FILES < <(find "$scen_dir" -maxdepth 1 -name "*.map")

    if [[ ${#MAP_FILES[@]} -ne 1 ]]; then
        echo
        echo "Skipping dataset without single map: $dataset"
        continue
    fi

    MAP_FILE="${MAP_FILES[0]}"

    or_res="$(run_one "$ORTOOLS_BIN" "$MAP_FILE" "$scen")"
    CURRENT_RUN=$((CURRENT_RUN + 1))
    progress_bar "$CURRENT_RUN" "$TOTAL_RUNS"

    stg2_res="$(run_one "$CPLEX_BIN" "$MAP_FILE" "$scen")"
    CURRENT_RUN=$((CURRENT_RUN + 1))
    progress_bar "$CURRENT_RUN" "$TOTAL_RUNS"

    stg3_res="$(run_one "$CPLEX_STG3_BIN" "$MAP_FILE" "$scen")"
    CURRENT_RUN=$((CURRENT_RUN + 1))
    progress_bar "$CURRENT_RUN" "$TOTAL_RUNS"

    IFS='|' read -r or_cost or_makespan or_time <<< "$or_res"
    IFS='|' read -r stg2_cost stg2_makespan stg2_time <<< "$stg2_res"
    IFS='|' read -r stg3_cost stg3_makespan stg3_time <<< "$stg3_res"

    [[ -z "$or_cost" ]] && or_cost="-"
    [[ -z "$or_makespan" ]] && or_makespan="-"
    [[ -z "$or_time" ]] && or_time="-"

    [[ -z "$stg2_cost" ]] && stg2_cost="-"
    [[ -z "$stg2_makespan" ]] && stg2_makespan="-"
    [[ -z "$stg2_time" ]] && stg2_time="-"

    [[ -z "$stg3_cost" ]] && stg3_cost="-"
    [[ -z "$stg3_makespan" ]] && stg3_makespan="-"
    [[ -z "$stg3_time" ]] && stg3_time="-"

    cost_match="$(match_status_3 "$or_cost" "$stg2_cost" "$stg3_cost")"
    makespan_match="$(match_status_3 "$or_makespan" "$stg2_makespan" "$stg3_makespan")"

    speedup_or_cplex="$(calc_ratio "$or_time" "$stg2_time")"
    speedup_or_stg3="$(calc_ratio "$or_time" "$stg3_time")"
    rate_cplex_stg3="$(calc_ratio "$stg2_time" "$stg3_time")"

    echo "| \`$dataset\` | \`$(basename "$scen")\` | $cost_match | $makespan_match | $or_time | $stg2_time | $stg3_time | $speedup_or_cplex | $speedup_or_stg3 | $rate_cplex_stg3 |" >> "$OUT_MD"
done

rm -f "$TMP_LOG"

echo
echo "Benchmark complete"
echo "Results written to: $OUT_MD"
