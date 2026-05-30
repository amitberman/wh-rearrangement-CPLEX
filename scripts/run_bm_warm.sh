#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 5 ]]; then
    echo "Usage: $0 <root_directory> [timeout_sec=60] [max_scenarios=0] [out_md=warmstart_results.md] [cold_mode=full|reuse]"
    exit 1
fi

ROOT_DIR="$1"
TIMEOUT_SEC="${2:-60}"
MAX_SCENARIOS="${3:-0}"
OUT_MD="${4:-warmstart_results.md}"
COLD_MODE="${5:-full}"

case "$COLD_MODE" in
    reuse)
        COLD_REUSE_MODEL=1
        ;;
    full)
        COLD_REUSE_MODEL=0
        ;;
    *)
        echo "Error: invalid cold_mode '$COLD_MODE' (expected 'reuse' or 'full')."
        exit 1
        ;;
esac

BIN="./build_cplex/MAWR"

if [[ ! -x "$BIN" ]]; then
    echo "Error: binary not found or not executable: $BIN"
    exit 1
fi

if [[ ! -d "$ROOT_DIR" ]]; then
    echo "Error: directory '$ROOT_DIR' does not exist."
    exit 1
fi

mapfile -t SCEN_FILES < <(find "$ROOT_DIR" -type f -name "*.scen" | sort)
if [[ "$MAX_SCENARIOS" -gt 0 ]]; then
    SCEN_FILES=("${SCEN_FILES[@]:0:$MAX_SCENARIOS}")
fi

if [[ ${#SCEN_FILES[@]} -eq 0 ]]; then
    echo "No scenarios found under '$ROOT_DIR'."
    exit 1
fi

TMP_WARM_LOG="$(mktemp)"
TMP_COLD_LOG="$(mktemp)"
trap 'rm -f "$TMP_WARM_LOG" "$TMP_COLD_LOG"' EXIT

extract_elapsed() {
    local f="$1"
    local val
    val="$(grep -E "Elapsed time:" "$f" | tail -n 1 | sed -E 's/.*Elapsed time: *([0-9]+(\.[0-9]+)?) sec.*/\1/' || true)"
    if [[ -z "$val" ]]; then
        echo "-"
    else
        echo "$val"
    fi
}

calc_speedup() {
    local cold="$1"
    local warm="$2"
    if [[ "$cold" == "-" || "$warm" == "-" ]]; then
        echo "-"
        return
    fi
    awk -v c="$cold" -v w="$warm" 'BEGIN { if (w > 0) printf "%.3fx", c / w; else print "-" }'
}

calc_ratio() {
    local num="$1"
    local den="$2"
    awk -v n="$num" -v d="$den" 'BEGIN { if (d > 0) printf "%.3f", n / d; else print "0.000" }'
}

{
    echo "# Warm-Start Benchmark Results"
    echo
    echo "- Root directory: \`$ROOT_DIR\`"
    echo "- Timeout: ${TIMEOUT_SEC} sec"
    echo "- Max scenarios: ${MAX_SCENARIOS}"
    echo "- Total scenarios: ${#SCEN_FILES[@]}"
    echo "- Cold mode: \`$COLD_MODE\`"
    echo "- Binary: \`$BIN\`"
    echo
    echo "| Dataset | Scenario | Warm Time (sec) | Cold Time (sec) | Cold / Warm | Warm Basis Loads | Warm Diff Updates | Warm Rebuilds | Warm Rebuild Ratio | Cold Diff Updates | Cold Rebuilds |"
    echo "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
} > "$OUT_MD"

for scen in "${SCEN_FILES[@]}"; do
    scen_dir="$(dirname "$scen")"
    dataset="$(basename "$scen_dir")"

    mapfile -t MAP_FILES < <(find "$scen_dir" -maxdepth 1 -name "*.map")
    if [[ ${#MAP_FILES[@]} -ne 1 ]]; then
        echo "Skipping '$dataset' (expected exactly one .map in dataset folder)."
        continue
    fi
    map_file="${MAP_FILES[0]}"

    MAWR_CPLEX_WARM_START=1 MAWR_CPLEX_REUSE_MODEL=1 "$BIN" \
        -m "$map_file" \
        -s "$scen" \
        -a NATCBS \
        -t "$TIMEOUT_SEC" \
        -o /tmp/mawr_warm_eval.csv \
        --v 2 > "$TMP_WARM_LOG" 2>&1 || true

    MAWR_CPLEX_WARM_START=0 MAWR_CPLEX_REUSE_MODEL="$COLD_REUSE_MODEL" "$BIN" \
        -m "$map_file" \
        -s "$scen" \
        -a NATCBS \
        -t "$TIMEOUT_SEC" \
        -o /tmp/mawr_cold_eval.csv \
        --v 2 > "$TMP_COLD_LOG" 2>&1 || true

    warm_time="$(extract_elapsed "$TMP_WARM_LOG")"
    cold_time="$(extract_elapsed "$TMP_COLD_LOG")"
    speedup="$(calc_speedup "$cold_time" "$warm_time")"

    warm_basis_loads="$(grep -c "\[CPLEX\] Warm start basis loaded." "$TMP_WARM_LOG" || true)"
    warm_diff_updates="$(grep -c "\[CPLEX\] Applied differential update" "$TMP_WARM_LOG" || true)"
    warm_rebuilds="$(grep -c "\[CPLEX\] Rebuilt network model" "$TMP_WARM_LOG" || true)"
    cold_diff_updates="$(grep -c "\[CPLEX\] Applied differential update" "$TMP_COLD_LOG" || true)"
    cold_rebuilds="$(grep -c "\[CPLEX\] Rebuilt network model" "$TMP_COLD_LOG" || true)"

    total_warm_changes=$((warm_diff_updates + warm_rebuilds))
    warm_rebuild_ratio="$(calc_ratio "$warm_rebuilds" "$total_warm_changes")"

    echo "| \`$dataset\` | \`$(basename "$scen")\` | $warm_time | $cold_time | $speedup | $warm_basis_loads | $warm_diff_updates | $warm_rebuilds | $warm_rebuild_ratio | $cold_diff_updates | $cold_rebuilds |" >> "$OUT_MD"
done

echo "Warm-start benchmark complete: $OUT_MD"
