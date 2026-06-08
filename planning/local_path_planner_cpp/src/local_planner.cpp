#include "local_path_planner_cpp/local_planner.hpp"

#include <algorithm>
#include <cmath>

namespace local_path_planner_cpp
{

std::vector<double> velocity_decel_to_target(
    double v0, double v_target, double L,
    const std::vector<double> & stations, double a_max)
{
  std::vector<double> out;
  out.reserve(stations.size());
  // Constant-decel ramp from v0 → v_target across lookahead L. If v0 ≤ v_target
  // there's nothing to brake for; just return v_target.
  if (v0 <= v_target || L <= 1e-3) {
    out.assign(stations.size(), v_target);
    return out;
  }
  // a = (v_target² - v0²) / (2·L) — solve for the (positive) deceleration
  // magnitude, clamp to a_max so we never overshoot the chassis limit.
  const double a_needed = (v0*v0 - v_target*v_target) / (2.0 * L);
  const double a = std::min(a_needed, a_max);
  for (double s : stations) {
    // v(s)² = max(v_target², v0² - 2·a·s)  ⇒  v(s) = sqrt(...)
    const double v2 = std::max(v_target*v_target, v0*v0 - 2.0 * a * s);
    out.push_back(std::sqrt(std::max(0.0, v2)));
  }
  return out;
}

std::vector<double> velocity_decel_to_stop(
    double v0, double L,
    const std::vector<double> & stations, double a_max)
{
  std::vector<double> out;
  out.reserve(stations.size());
  if (L <= 1e-3) { out.assign(stations.size(), 0.0); return out; }
  // a needed to reach 0 by s=L: a = v0² / (2·L). Clamp to a_max.
  const double a_needed = (v0*v0) / (2.0 * L);
  const double a = std::min(a_needed, a_max);
  for (double s : stations) {
    const double v2 = std::max(0.0, v0*v0 - 2.0 * a * s);
    out.push_back(std::sqrt(v2));
  }
  return out;
}

}  // namespace local_path_planner_cpp
