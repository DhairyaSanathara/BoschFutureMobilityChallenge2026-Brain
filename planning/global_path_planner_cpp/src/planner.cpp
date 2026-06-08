#include "global_path_planner_cpp/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace global_path_planner_cpp
{

namespace
{

// Reconstruct the start-to-goal node-id sequence by walking `came_from`
// backwards from goal. Returns false (and leaves out_path empty) if goal
// is unreachable in the map.
bool reconstruct(
    const std::string & start, const std::string & goal,
    const std::unordered_map<std::string, std::string> & came_from,
    std::vector<std::string> & out_path)
{
  out_path.clear();
  if (came_from.find(goal) == came_from.end() && start != goal) {
    return false;       // goal never expanded
  }
  std::string cur = goal;
  while (true) {
    out_path.push_back(cur);
    if (cur == start) break;
    auto it = came_from.find(cur);
    if (it == came_from.end()) { out_path.clear(); return false; }
    cur = it->second;
    if (out_path.size() > came_from.size() + 2) {
      out_path.clear(); return false;        // cycle defence
    }
  }
  std::reverse(out_path.begin(), out_path.end());
  return true;
}

double euclid(const GraphNode & a, const GraphNode & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

double segment_point_distance(
    double px, double py,
    double ax, double ay,
    double bx, double by)
{
  // Standard projection-onto-segment formula. t clamped to [0,1] so endpoints
  // become valid nearest points instead of being projected to the infinite line.
  const double dx = bx - ax;
  const double dy = by - ay;
  const double L2 = dx * dx + dy * dy;
  double t = (L2 > 0.0) ? ((px - ax) * dx + (py - ay) * dy) / L2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  const double cx = ax + t * dx;
  const double cy = ay + t * dy;
  const double ex = px - cx;
  const double ey = py - cy;
  return std::sqrt(ex * ex + ey * ey);
}

}  // namespace

bool plan(const Graph & graph,
          const std::string & start, const std::string & goal,
          Algo algo,
          std::vector<std::string> & out_path,
          double & out_cost)
{
  out_path.clear();
  out_cost = std::numeric_limits<double>::infinity();

  if (!graph.node(start) || !graph.node(goal)) return false;
  if (start == goal) { out_path.push_back(start); out_cost = 0.0; return true; }

  // g_score[id] = best known cost from start to id.
  std::unordered_map<std::string, double>      g;
  std::unordered_map<std::string, std::string> came_from;
  g.reserve(graph.n_nodes());
  came_from.reserve(graph.n_nodes());
  g[start] = 0.0;

  // priority_queue<pair<f_score, id>> with min-heap behaviour via std::greater.
  using QEntry = std::pair<double, std::string>;
  std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>> open;

  const GraphNode * goal_node = graph.node(goal);
  auto heuristic = [&](const std::string & id) -> double {
    if (algo == Algo::Dijkstra) return 0.0;
    const auto * n = graph.node(id);
    return (n && goal_node) ? euclid(*n, *goal_node) : 0.0;
  };

  open.emplace(heuristic(start), start);

  while (!open.empty()) {
    auto [f, cur] = open.top(); open.pop();
    if (cur == goal) {
      out_cost = g[goal];
      return reconstruct(start, goal, came_from, out_path);
    }
    // Stale queue entry (we already found a shorter path to `cur`).
    const double g_cur = g[cur];
    if (f > g_cur + heuristic(cur) + 1e-9) continue;

    for (const auto & e : graph.edges_from(cur)) {
      const double tentative = g_cur + e.cost;
      auto it = g.find(e.target);
      if (it == g.end() || tentative < it->second) {
        g[e.target] = tentative;
        came_from[e.target] = cur;
        open.emplace(tentative + heuristic(e.target), e.target);
      }
    }
  }
  return false;        // goal unreachable
}

double distance_to_path(const Graph & graph,
                        double x, double y,
                        const std::vector<std::string> & path)
{
  if (path.size() < 2) return std::numeric_limits<double>::infinity();
  double min_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto * a = graph.node(path[i]);
    const auto * b = graph.node(path[i + 1]);
    if (!a || !b) return std::numeric_limits<double>::infinity();
    const double d = segment_point_distance(x, y, a->x, a->y, b->x, b->y);
    if (d < min_d) min_d = d;
  }
  return min_d;
}

}  // namespace global_path_planner_cpp
