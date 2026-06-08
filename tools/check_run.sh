#!/usr/bin/env bash
# bfmc_autonomy/tools/check_run.sh
#
# Race-day go/no-go: measure key topic rates and PASS/FAIL against thresholds.
# Two modes:
#   • DEFAULT — read /system/health/metrics once (requires the stack to have
#     been launched with enable_runtime_validation:=true).
#   • FALLBACK — if no health_monitor data is available within HEALTH_TIMEOUT
#     seconds, fall back to direct `ros2 topic hz` sampling for HZ_SAMPLE
#     seconds. Slower but works with any launch.
#
# Usage:
#   tools/check_run.sh                    # default mode
#   tools/check_run.sh --hz [duration]    # force fallback mode
#   tools/check_run.sh --help
#
# Exit status: 0 = all checks pass, 1 = any check fails, 2 = no data at all.

set -u

HEALTH_TIMEOUT="${HEALTH_TIMEOUT:-3}"
HZ_SAMPLE="${HZ_SAMPLE:-5}"
FORCE_HZ="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --hz)
      FORCE_HZ="true"
      if [[ $# -ge 2 && "$2" =~ ^[0-9]+$ ]]; then HZ_SAMPLE="$2"; shift; fi
      ;;
    -h|--help)
      sed -n '1,/^set -u/p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "unknown arg: $1" >&2; exit 2 ;;
  esac
  shift
done

# Thresholds (Hz)
declare -A THRESH=(
  [vehicle_state]=80
  [tf]=80
  [imu]=60
  [camera]=25
  [lane]=20
)

# Pretty-printers
bold()   { printf '\033[1m%s\033[0m' "$1"; }
green()  { printf '\033[32m%s\033[0m' "$1"; }
red()    { printf '\033[31m%s\033[0m' "$1"; }
yellow() { printf '\033[33m%s\033[0m' "$1"; }

require_ros2() {
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "$(red ERROR): 'ros2' not in PATH — did you source your workspace?" >&2
    exit 2
  fi
}

# Returns the rate as plain float, or empty string if not available.
read_via_health() {
  # /system/health/metrics is Float32MultiArray[18]; layout in
  # health_monitor_node.hpp. Layout indices:
  #   0 vehicle_state  1 tf  2 camera  3 lane  4 imu  16 warning_count
  local raw
  raw="$(timeout "${HEALTH_TIMEOUT}" ros2 topic echo --once \
           /system/health/metrics 2>/dev/null)" || return 1

  # Pull the bracketed data array from the YAML-ish dump.
  local arr
  arr="$(printf '%s\n' "$raw" | awk '/^data:/{flag=1} flag{print}' \
           | tr '\n' ' ' | sed 's/.*\[\([^]]*\)\].*/\1/')"
  [[ -z "$arr" ]] && return 1
  IFS=',' read -r -a vals <<<"$arr"
  for i in "${!vals[@]}"; do vals[$i]="${vals[$i]// /}"; done

  if [[ ${#vals[@]} -lt 17 ]]; then return 1; fi

  HEALTH_VEHICLE_STATE="${vals[0]}"
  HEALTH_TF="${vals[1]}"
  HEALTH_CAMERA="${vals[2]}"
  HEALTH_LANE="${vals[3]}"
  HEALTH_IMU="${vals[4]}"
  HEALTH_WARNING_COUNT="${vals[16]}"
  return 0
}

# Returns the rate as plain float, or empty string if not available.
read_via_hz() {
  local topic="$1"
  timeout "${HZ_SAMPLE}" ros2 topic hz "$topic" 2>&1 \
    | grep -oE 'average rate:[[:space:]]*[0-9.]+' \
    | tail -1 | awk '{print $NF}'
}

# Pretty-print a (name, value, threshold) row and mutate ALL_PASS.
ALL_PASS="true"
check_row() {
  local name="$1" val="$2" thr="$3"
  printf "  %-15s " "$name"
  if [[ -z "$val" || "$val" == "0.0" || "$val" == "0" ]]; then
    printf "%s   (target >%s Hz)\n" "$(red 'NO DATA')" "$thr"
    ALL_PASS="false"
    return
  fi
  # awk-based float comparison, no external bc dependency.
  local pass
  pass="$(awk -v v="$val" -v t="$thr" 'BEGIN{print (v>t) ? "1" : "0"}')"
  if [[ "$pass" == "1" ]]; then
    printf "%6.1f Hz   %s (target >%s)\n" "$val" "$(green PASS)" "$thr"
  else
    printf "%6.1f Hz   %s (target >%s)\n" "$val" "$(red  FAIL)" "$thr"
    ALL_PASS="false"
  fi
}

main() {
  require_ros2

  echo
  echo "=== $(bold "SYSTEM HEALTH") ==="
  echo

  local source=""
  local vs tf imu cam lane wc=""

  if [[ "$FORCE_HZ" == "false" ]] && read_via_health; then
    source="/system/health/metrics (health_monitor)"
    vs="$HEALTH_VEHICLE_STATE"; tf="$HEALTH_TF"; imu="$HEALTH_IMU"
    cam="$HEALTH_CAMERA";       lane="$HEALTH_LANE"
    wc="$HEALTH_WARNING_COUNT"
  else
    echo "$(yellow '[fallback]') no health_monitor — sampling ros2 topic hz for ${HZ_SAMPLE}s"
    echo
    source="ros2 topic hz (${HZ_SAMPLE}s windows, sequential)"
    vs="$(read_via_hz /vehicle/state)"
    tf="$(read_via_hz /tf)"
    imu="$(read_via_hz /microros/imu_raw)"
    cam="$(read_via_hz /camera/camera/color/image_raw)"
    lane="$(read_via_hz /perception/lane_geometry)"
  fi

  echo "source: ${source}"
  echo
  check_row "vehicle_state" "$vs"   "${THRESH[vehicle_state]}"
  check_row "tf"            "$tf"   "${THRESH[tf]}"
  check_row "imu"           "$imu"  "${THRESH[imu]}"
  check_row "camera"        "$cam"  "${THRESH[camera]}"
  check_row "lane"          "$lane" "${THRESH[lane]}"
  echo
  if [[ -n "$wc" ]]; then
    if [[ "$wc" == "0" || "$wc" == "0.0" ]]; then
      printf "  %-15s %s\n" "warnings" "$(green 0)"
    else
      printf "  %-15s %s\n" "warnings" "$(yellow "$wc")"
    fi
  fi

  echo
  if [[ "$ALL_PASS" == "true" ]]; then
    echo "OVERALL: $(green PASS)"
    exit 0
  else
    echo "OVERALL: $(red FAIL)"
    exit 1
  fi
}

main "$@"
