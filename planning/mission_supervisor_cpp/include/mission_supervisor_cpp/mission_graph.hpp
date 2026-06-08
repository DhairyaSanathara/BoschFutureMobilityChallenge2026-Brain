#ifndef MISSION_SUPERVISOR_CPP__MISSION_GRAPH_HPP_
#define MISSION_SUPERVISOR_CPP__MISSION_GRAPH_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mission_supervisor_cpp
{

// Read-only minimal GraphML loader.
// We don't reuse global_path_planner_cpp::Graph (which is fine but heavier) —
// the supervisor only needs x/y lookup + has_edge(src,tgt). Keeping this
// standalone means the mission supervisor builds even if global_path_planner_cpp
// is later moved or renamed.
struct GraphNode
{
  std::string id;
  double      x{0.0};
  double      y{0.0};
};

class MissionGraph
{
public:
  // Returns true on success; populates `error` with a one-line reason on failure.
  bool load(const std::string & graphml_path, std::string & error);

  bool has_node(const std::string & id) const;
  bool has_edge(const std::string & src, const std::string & tgt) const;
  const GraphNode * node(const std::string & id) const;

  // BFS shortest path (in hops) from src → tgt. Returns the full ordered
  // node list including both endpoints, or an empty vector if no path exists.
  // BFS over hops is appropriate here because the BFMC graph edges are roughly
  // equal in length (junction-to-junction); a Dijkstra by Euclidean cost would
  // sometimes prefer a longer-but-more-aligned route, which is undesirable
  // when we're auto-filling a mission gap and want minimum graph deviation.
  std::vector<std::string> shortest_path(const std::string & src,
                                          const std::string & tgt) const;

  std::size_t n_nodes() const { return nodes_.size(); }
  std::size_t n_edges() const { return n_edges_; }

private:
  std::unordered_map<std::string, GraphNode>                       nodes_;
  // Sparse undirected adjacency? No — keep both directions because the BFMC
  // map has one-way edges. has_edge(a,b) means "edge a→b exists in file".
  std::unordered_map<std::string, std::unordered_set<std::string>> adj_;
  std::size_t                                                      n_edges_{0};
};

}  // namespace mission_supervisor_cpp

#endif  // MISSION_SUPERVISOR_CPP__MISSION_GRAPH_HPP_
