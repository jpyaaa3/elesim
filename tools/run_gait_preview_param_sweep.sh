#!/usr/bin/env bash
# Offline gait-preview param sweep on existing logs, then optional live validation of top-N.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

SOURCE_CONFIG="${SOURCE_CONFIG:-config.ini}"
LOG_DIR="${LOG_DIR:-logs/walking_baseline}"
OUT_DIR="${OUT_DIR:-results/gait_preview_tune}"
TOP_K="${TOP_K:-5}"
LIVE_TOP_N="${LIVE_TOP_N:-3}"
TRIALS="${TRIALS:-1}"
DRY_RUN="${DRY_RUN:-0}"

mkdir -p "$OUT_DIR"

echo "=== Phase 1: offline sweep on v8 gait-preview logs ==="
PYTHONPATH=. python tools/sweep_gait_preview_params.py \
  --log-dir "$LOG_DIR" \
  --notes-filter v8 \
  --top-k "$TOP_K" \
  --out "$OUT_DIR/sweep_offline.json" \
  --write-top-configs \
  --source-config "$SOURCE_CONFIG" \
  --config-out-dir "$OUT_DIR/configs"

if [[ "$DRY_RUN" == "1" ]]; then
  echo "DRY_RUN=1 — skipping live validation"
  exit 0
fi

echo "=== Phase 2: live validation (top ${LIVE_TOP_N}, ${TRIALS} trial each) ==="
python - "$OUT_DIR/sweep_offline.json" "$LIVE_TOP_N" <<'PY'
import json, sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
for row in payload.get("top", [])[: int(sys.argv[2])]:
    print(row["tag"])
PY
mapfile -t TAGS < <(python - "$OUT_DIR/sweep_offline.json" "$LIVE_TOP_N" <<'PY'
import json, sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
for row in payload.get("top", [])[: int(sys.argv[2])]:
    print(row["tag"])
PY
)

BATCH_LOG_DIR="${LOG_DIR}/_gait_tune_validate"
mkdir -p "$BATCH_LOG_DIR"

for tag in "${TAGS[@]}"; do
  cfg="$OUT_DIR/configs/gait_tune_${tag}.ini"
  if [[ ! -f "$cfg" ]]; then
    echo "missing config for tag=$tag" >&2
    continue
  fi
  echo "--- live validate tag=$tag ---"
  PYTHONPATH=. python tools/run_walking_baseline_batch.py \
    --config "$cfg" \
    --gaze preview \
    --motion forward \
    --preset neutral \
    --max-duration 30 \
    --stop-at-standoff 0.85 \
    --pose-settle-s 2.0 \
    --video-preroll-s 2.0 \
    --sim-warmup-s 180 \
    --perception-warmup-s 30 \
    --restart-host \
    --trials "$TRIALS" \
    --run-id-stem "neutral_forward_gait_tune_${tag}" \
    --log-dir "$LOG_DIR" \
    --batch-log-dir "$BATCH_LOG_DIR" \
    --notes "gait tune validate ${tag}"
done

echo "=== Phase 3: analyze validation runs ==="
python - "$LOG_DIR" "$OUT_DIR" <<'PY'
import json, subprocess, sys
from pathlib import Path

log_dir = Path(sys.argv[1])
out_dir = Path(sys.argv[2])
sweep = json.loads((out_dir / "sweep_offline.json").read_text(encoding="utf-8"))
rows = []
for cand in sweep.get("top", []):
    tag = cand["tag"]
    run_id = f"neutral_forward_gait_tune_{tag}_001"
    summary_path = log_dir / f"{run_id}_summary.json"
    if not summary_path.is_file():
        continue
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    rows.append(
        {
            "tag": tag,
            "run_id": run_id,
            "offline_proxy_v_rms": cand["proxy_v_rms"],
            "live_v_rms": summary.get("v_rms"),
            "params": {
                "phase_offset": cand["phase_offset"],
                "horizon_s": cand["horizon_s"],
                "scale": cand["scale"],
            },
        }
    )
out = {"validation": rows}
(out_dir / "sweep_validation.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
print(json.dumps(out, indent=2))
PY

echo "Done. See $OUT_DIR/sweep_offline.json and $OUT_DIR/sweep_validation.json"
