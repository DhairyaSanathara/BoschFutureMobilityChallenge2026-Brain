#!/usr/bin/env bash
# =============================================================================
# probe_engine.sh — sanity-check the deployed YOLOv8 TRT engine.
# Reports input/output tensor shapes + per-iteration latency.
# =============================================================================
set -euo pipefail

ENGINE="${1:-}"
ITERS="${2:-50}"

if [ -z "${ENGINE}" ]; then
  echo "usage: $0 <engine_path> [iterations]" >&2
  exit 1
fi

if [ ! -f "${ENGINE}" ]; then
  echo "engine not found: ${ENGINE}" >&2
  exit 1
fi

TRTEXEC="$(command -v trtexec || echo /usr/src/tensorrt/bin/trtexec)"
if [ ! -x "${TRTEXEC}" ]; then
  echo "trtexec not found"
  exit 1
fi

echo "[probe] engine: ${ENGINE}"
echo "[probe] iters:  ${ITERS}"

"${TRTEXEC}" \
  --loadEngine="${ENGINE}" \
  --iterations="${ITERS}" \
  --warmUp=200 \
  --duration=0 \
  --avgRuns="${ITERS}" \
  --useSpinWait \
  --noDataTransfers \
  2>&1 | tail -40
