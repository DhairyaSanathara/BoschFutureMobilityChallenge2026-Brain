#include "global_path_planner_cpp/graph.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace global_path_planner_cpp
{

namespace
{
constexpr double EPS = 1e-3;   // floor on speed_limit when computing cost
}

void Graph::add_node(GraphNode n)
{
  nodes_[n.id] = std::move(n);
}

void Graph::add_edge(GraphEdge e)
{
  // Compute length + cost up front. If either endpoint is missing we drop
  // the edge silently — the parser will have already logged the malformed
  // edge.
  auto it_src = nodes_.find(e.source);
  auto it_tgt = nodes_.find(e.target);
  if (it_src == nodes_.end() || it_tgt == nodes_.end()) return;

  const double dx = it_tgt->second.x - it_src->second.x;
  const double dy = it_tgt->second.y - it_src->second.y;
  e.length = std::sqrt(dx * dx + dy * dy);
  e.cost   = e.length / std::max(e.speed_limit, EPS);

  adj_[e.source].push_back(std::move(e));
}

const GraphNode * Graph::node(const std::string & id) const
{
  auto it = nodes_.find(id);
  return (it == nodes_.end()) ? nullptr : &it->second;
}

const std::vector<GraphEdge> & Graph::edges_from(const std::string & id) const
{
  auto it = adj_.find(id);
  return (it == adj_.end()) ? empty_edge_list_ : it->second;
}

std::string Graph::nearest_node(double x, double y) const
{
  std::string best_id;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (const auto & [id, n] : nodes_) {
    const double dx = n.x - x;
    const double dy = n.y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; best_id = id; }
  }
  return best_id;
}

std::size_t Graph::n_edges() const
{
  std::size_t total = 0;
  for (const auto & [_, v] : adj_) total += v.size();
  return total;
}

}  // namespace global_path_planner_cpp
