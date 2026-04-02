#!/usr/bin/env bash
set -u

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <root_directory>"
    exit 1
fi

ROOT_DIR="$1"

ORTOOLS_BIN="./build_ortools/MAWR"
CPLEX_W_LB_BIN="./build_cplex_w_lb/MAWR"
CPLEX_2STAGE_BIN="./build_cplex/MAWR"

OUT_MD="benchmark_results.md"
TMP_LOG="$(mktemp)"

if [[ ! -d "$ROOT_DIR" ]]; then
    echo "Error: directory '$ROOT_DIR' does not exist."
    exit 1
fi

for bin in "$ORTOOLS_BIN" "$CPLEX_W_LB_BIN" "$CPLEX_2STAGE_BIN"; do
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

    echo "${min_cost}|${makespan}|${elapsed_time}|${exit_code}"
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

match_status() {
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
    echo "| Dataset | Scenario | Cost Match | Makespan Match | Time (OR) | Time (CPLEX-1STAGE) | Time (CPLEX-2STAGE) | Speedup OR/CPLEX | Rate 1STAGE/2STAGE |"
    echo "|---|---|---|---|---:|---:|---:|---:|---:|"
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

    wlb_res="$(run_one "$CPLEX_W_LB_BIN" "$MAP_FILE" "$scen")"
    CURRENT_RUN=$((CURRENT_RUN + 1))
    progress_bar "$CURRENT_RUN" "$TOTAL_RUNS"

    two_res="$(run_one "$CPLEX_2STAGE_BIN" "$MAP_FILE" "$scen")"
    CURRENT_RUN=$((CURRENT_RUN + 1))
    progress_bar "$CURRENT_RUN" "$TOTAL_RUNS"

    IFS='|' read -r or_cost or_makespan or_time _ <<< "$or_res"
    IFS='|' read -r wlb_cost wlb_makespan wlb_time _ <<< "$wlb_res"
    IFS='|' read -r two_cost two_makespan two_time _ <<< "$two_res"

    [[ -z "$or_cost" ]] && or_cost="-"
    [[ -z "$or_makespan" ]] && or_makespan="-"
    [[ -z "$or_time" ]] && or_time="-"

    [[ -z "$wlb_cost" ]] && wlb_cost="-"
    [[ -z "$wlb_makespan" ]] && wlb_makespan="-"
    [[ -z "$wlb_time" ]] && wlb_time="-"

    [[ -z "$two_cost" ]] && two_cost="-"
    [[ -z "$two_makespan" ]] && two_makespan="-"
    [[ -z "$two_time" ]] && two_time="-"

    cost_match="$(match_status "$or_cost" "$wlb_cost" "$two_cost")"
    makespan_match="$(match_status "$or_makespan" "$wlb_makespan" "$two_makespan")"

    speedup_or_cplex="$(calc_ratio "$or_time" "$two_time")"
    rate_1stage_2stage="$(calc_ratio "$wlb_time" "$two_time")"

    echo "| \`$dataset\` | \`$(basename "$scen")\` | $cost_match | $makespan_match | $or_time | $wlb_time | $two_time | $speedup_or_cplex | $rate_1stage_2stage |" >> "$OUT_MD"
done

rm -f "$TMP_LOG"

echo
echo "Benchmark complete"
echo "Results written to: $OUT_MD"