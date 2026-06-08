#ifndef GLOBAL_PATH_PLANNER_CPP__ZONE_MAP_HPP_
#define GLOBAL_PATH_PLANNER_CPP__ZONE_MAP_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "global_path_planner_cpp/graph.hpp"

namespace global_path_planner_cpp
{

// Out-of-band side-car (zones.yaml) — see maps/zones.yaml for the schema.
struct ZoneMap
{
  std::unordered_map<std::string, std::string> node_types;     // node id → type
  std::unordered_map<std::string, std::string> semaphores;     // node id → V2X semaphore id
  // edge-key string is `src + ">" + tgt` so we can index directly with one
  // hash lookup per edge probe.
  std::unordered_set<std::string>              highway_edges;
  std::unordered_map<std::string, double>      edge_speed_overrides;
  double                                       default_edge_speed_limit{0.20};

  // Loads the YAML side-car. Returns false (and sets `error`) on file-open
  // or YAML-parse failure. Silently skips unknown / malformed entries.
  bool load(const std::string & yaml_path, std::string & error);

  // Re-key an edge into "src>tgt" form for the highway / override lookups.
  static std::string edge_key(const std::string & src, const std::string & tgt);

  // After load(), patches every edge's speed_limit (and therefore cost) in
  // the graph to match `edge_speed_overrides` / `default_edge_speed_limit`.
  void apply_speed_limits_to(Graph & g) const;

  // Lookup helpers.
  const std::string * node_type(const std::string & id) const;
  const std::string * semaphore_for(const std::string & node_id) const;
  bool                edge_is_highway(const std::string & src,
                                      const std::string & tgt) const;
};

}  // namespace global_path_planner_cpp

#endif  // GLOBAL_PATH_PLANNER_CPP__ZONE_MAP_HPP_
