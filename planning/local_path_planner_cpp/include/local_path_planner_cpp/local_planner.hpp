#ifndef LOCAL_PATH_PLANNER_CPP__LOCAL_PLANNER_HPP_
#define LOCAL_PATH_PLANNER_CPP__LOCAL_PLANNER_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace local_path_planner_cpp
{

// A single waypoint of the published local trajectory. Field order matches the
// Python local_path_planner JSON schema exactly so the downstream
// trajectory_generator continues to consume our output without modification.
struct TrajectoryPoint
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double curvature{0.0};
  double target_speed{0.0};
};

// Quadratic-poly helpers shared by lane_follow / decel / overtake. Poly is
// stored (a, b, c) so y = a + b*x + c*x²  (this matches the Python
// `eval_quadratic` convention — see local_path_planner_node.py).
using Poly3 = std::array<double, 3>;

inline double eval_quadratic     (const Poly3 & p, double x) { return p[0] + p[1]*x + p[2]*x*x; }
inline double eval_quadratic_d   (const Poly3 & p, double x) { return p[1] + 2.0*p[2]*x; }
inline double heading_quadratic  (const Poly3 & p, double x) { return std::atan(eval_quadratic_d(p, x)); }
inline double curvature_quadratic(const Poly3 & p, double x)
{
  const double dy = eval_quadratic_d(p, x);
  const double d2y = 2.0 * p[2];
  const double denom = std::pow(1.0 + dy*dy, 1.5);
  return (denom > 1e-12) ? d2y / denom : 0.0;
}

// Sigmoid lateral offset y(x) for overtake. Returns (y_off, dy_off, d2y_off).
inline void lateral_offset_sigmoid(double x, double delta_y, double d_trans,
                                   double & y, double & dy, double & d2y)
{
  // Logistic-style smooth transition centred at x = d_trans/2 with steepness
  // such that ~99% of delta_y is reached by x = d_trans.
  const double k = 6.0 / std::max(d_trans, 1e-3);
  const double xs = x - 0.5 * d_trans;
  const double e  = std::exp(-k * xs);
  const double s  = 1.0 / (1.0 + e);
  const double ds = k * e / std::pow(1.0 + e, 2.0);
  const double d2s = k * k * e * (e - 1.0) / std::pow(1.0 + e, 3.0);
  y   = delta_y * s;
  dy  = delta_y * ds;
  d2y = delta_y * d2s;
}

// Speed-profile helper: linear decel-to-target along the lookahead window.
std::vector<double> velocity_decel_to_target(
    double v0, double v_target, double L,
    const std::vector<double> & stations, double a_max);

// Speed-profile helper: square-root decel-to-zero along the lookahead window.
std::vector<double> velocity_decel_to_stop(
    double v0, double L,
    const std::vector<double> & stations, double a_max);

}  // namespace local_path_planner_cpp

#endif  // LOCAL_PATH_PLANNER_CPP__LOCAL_PLANNER_HPP_
