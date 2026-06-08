#ifndef GLOBAL_PATH_PLANNER_CPP__GRAPH_HPP_
#define GLOBAL_PATH_PLANNER_CPP__GRAPH_HPP_

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace global_path_planner_cpp
{

// One vertex on the BFMC competition map. Coordinates are in map metres
// (whatever the GraphML file ships — we don't transform).
struct GraphNode
{
  std::string id;
  double      x{0.0};
  double      y{0.0};
  std::string type;   // free-form (e.g. "intersection", "parking", "roundabout")
};

// One directed edge between two graph nodes. `weight()` is set at insertion
// time so the planner doesn't recompute it per Dijkstra/A* probe.
struct GraphEdge
{
  std::string source;
  std::string target;
  double      speed_limit{1.0};   // m/s, default 1 m/s if not specified
  std::string road_type{"default"};
  int         priority{1};
  std::string lane_direction{"forward"};
  std::string traffic_light_id;
  double      length{0.0};        // Euclidean distance between endpoints (m)

  // Cost = length / max(speed_limit, EPS) — i.e. travel time minimisation.
  // Pre-computed so the planner's inner loop is O(1) per edge.
  double cost{0.0};
};

class Graph
{
public:
  void add_node(GraphNode n);

  // Computes `length` and `cost` from current node coordinates + speed_limit
  // before insertion. The endpoint nodes MUST already exist (logged + skipped
  // otherwise).
  void add_edge(GraphEdge e);

  // O(1) node lookup. Returns nullptr if id absent.
  const GraphNode * node(const std::string & id) const;

  // O(1) outgoing-edge list lookup. Returns empty vector reference if id
  // has no outgoing edges (e.g. terminal nodes).
  const std::vector<GraphEdge> & edges_from(const std::string & id) const;

  // Brute-force nearest-node search by Euclidean distance. BFMC maps have
  // < 500 nodes; KD-tree would be overkill at this scale. O(N) per call.
  // Returns empty string if graph is empty.
  std::string nearest_node(double x, double y) const;

  // Iteration support for diagnostics and reachability checks.
  const std::unordered_map<std::string, GraphNode> & nodes() const { return nodes_; }
  std::size_t n_nodes() const { return nodes_.size(); }
  std::size_t n_edges() const;

private:
  std::unordered_map<std::string, GraphNode>              nodes_;
  std::unordered_map<std::string, std::vector<GraphEdge>> adj_;
  std::vector<GraphEdge>                                  empty_edge_list_;  // returned by edges_from when no edges
};

}  // namespace global_path_planner_cpp

#endif  // GLOBAL_PATH_PLANNER_CPP__GRAPH_HPP_
