#!/usr/bin/env bash
# =============================================================================
# benchmark_latency.sh — quick verification of the C++ object detection node.
#
# Assumes the workspace is sourced and the RealSense camera (or a bag playback)
# is publishing on /camera/camera/color/image_raw + depth + camera_info.
#
# Run in a separate terminal, AFTER launching object_detection_node.
# Captures 20s of telemetry to the current directory.
# =============================================================================
set -euo pipefail

OUT_DIR="${1:-objdet_bench_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${OUT_DIR}"
cd "${OUT_DIR}"

echo "[bench] writing to ${PWD}"

# ---- topic rates (input + output) ------------------------------------------
( timeout 20s ros2 topic hz /camera/camera/color/image_raw  2>&1
) > rate_input_rgb.txt &

( timeout 20s ros2 topic hz /perception/objects 2>&1
) > rate_output_objects.txt &

# ---- output bandwidth -------------------------------------------------------
( timeout 20s ros2 topic bw /perception/objects 2>&1
) > bw_output_objects.txt &

# ---- Jetson SoC telemetry (CPU/GPU load, mem, EMC) --------------------------
if command -v tegrastats >/dev/null 2>&1; then
  ( timeout 20s tegrastats --interval 200 2>&1
  ) > tegrastats.txt &
else
  echo "tegrastats not on PATH — skipping SoC trace" > tegrastats.txt
fi

# ---- objdet's own health log ------------------------------------------------
# Health log line: "HEALTH [N frames | Us] fps=X avg=Yms p50=... p95=..."
# Capture the last 20s of node stderr via ros2 daemon -> /rosout topic.
( timeout 20s ros2 topic echo --no-arr /rosout 2>&1 \
   | grep -i "HEALTH" || true
) > health_lines.txt &

wait

echo "[bench] === input fps (rgb) ==="
tail -5 rate_input_rgb.txt
echo "[bench] === output fps (objects) ==="
tail -5 rate_output_objects.txt
echo "[bench] === output bw (objects) ==="
tail -5 bw_output_objects.txt
echo "[bench] === HEALTH lines from node ==="
tail -10 health_lines.txt
echo "[bench] === tegrastats sample (last line) ==="
tail -1 tegrastats.txt

echo "[bench] full traces in ${PWD}"
