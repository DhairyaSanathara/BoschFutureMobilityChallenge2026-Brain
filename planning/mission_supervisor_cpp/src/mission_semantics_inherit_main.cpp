// Standalone audit tool — for every node in the expanded mission, decide its
// region tag. Originally-listed nodes keep their tags from
// competition_semantics.yaml. Inserted (BFS-detour) nodes inherit from
// proximity (co-located BFMC junction cluster) or from forward/backward
// neighbours in the same region.
//
// Usage:
//   mission_semantics_inherit <graphml>
//                              <competition_semantics.yaml>
//                              <competition_mission_expanded.yaml>
//                              <competition_mission_original.yaml>
//                              <out_competition_semantics_expanded.yaml>
//                              <out_inheritance_report.csv>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "mission_supervisor_cpp/mission_graph.hpp"

using mission_supervisor_cpp::MissionGraph;
using mission_supervisor_cpp::GraphNode;

namespace {

// Proximity threshold for "co-located BFMC junction cluster" inheritance.
// BFMC's 11 co-located clusters are within ~1m of each other; 0.8m is safe.
constexpr double kClusterRadiusM = 0.80;

std::vector<std::string> load_yaml_path_(const std::string & path)
{
  std::vector<std::string> out;
  YAML::Node y = YAML::LoadFile(path);
  if (!y["mission"] || !y["mission"]["path"]) {
    throw std::runtime_error("missing mission.path in " + path);
  }
  for (const auto & n : y["mission"]["path"]) out.push_back(n.as<std::string>());
  return out;
}

std::unordered_map<std::string, std::string>
load_semantics_(const std::string & path)
{
  std::unordered_map<std::string, std::string> out;
  YAML::Node y = YAML::LoadFile(path);
  auto regs = y["regions"];
  if (!regs) return out;
  for (auto it = regs.begin(); it != regs.end(); ++it) {
    const std::string region = it->first.as<std::string>();
    const YAML::Node body = it->second;
    if (body["nodes"]) {
      for (const auto & n : body["nodes"]) out[n.as<std::string>()] = region;
    }
    if (body["node_ranges"]) {
      for (const auto & r : body["node_ranges"]) {
        if (r.size() < 2) continue;
        int lo = r[0].as<int>();
        int hi = r[1].as<int>();
        if (lo > hi) std::swap(lo, hi);
        for (int i = lo; i <= hi; ++i) out[std::to_string(i)] = region;
      }
    }
  }
  return out;
}

// Regions for which we propagate via co-location. crosswalk/intersection/
// roundabout are physically point-like (one or two co-located graph nodes
// per real-world feature); tunnel / low_visibility / no_lane / highway are
// spatial extents that we DO NOT want to bleed into adjacent nodes via
// proximity (they're explicitly enumerated per node or range in the YAML).
const std::set<std::string> kPropagatable = {
    "intersection", "roundabout", "crosswalk"};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 7) {
    std::cerr << "usage: " << argv[0]
              << " <graphml> <semantics.yaml> <mission_expanded.yaml> "
              << "<mission_original.yaml> <out_expanded.yaml> <out_report.csv>\n";
    return 64;
  }
  const std::string graphml         = argv[1];
  const std::string semantics_in    = argv[2];
  const std::string mission_exp     = argv[3];
  const std::string mission_orig    = argv[4];
  const std::string semantics_out   = argv[5];
  const std::string report_csv      = argv[6];

  MissionGraph g;
  std::string err;
  if (!g.load(graphml, err)) {
    std::cerr << "FATAL: graph load: " << err << "\n";
    return 64;
  }
  std::cerr << "graph: " << g.n_nodes() << " nodes\n";

  auto seed_tags = load_semantics_(semantics_in);
  auto path_exp  = load_yaml_path_(mission_exp);
  auto path_orig = load_yaml_path_(mission_orig);

  std::unordered_set<std::string> original_set(path_orig.begin(), path_orig.end());

  // Build per-region seed-node coordinate lists for proximity propagation.
  std::unordered_map<std::string, std::vector<const GraphNode *>> seeds_by_region;
  for (const auto & [nid, region] : seed_tags) {
    if (!kPropagatable.count(region)) continue;
    if (const auto * gn = g.node(nid)) seeds_by_region[region].push_back(gn);
  }

  // Resolve per-node region tag for every node in the expanded path.
  std::unordered_map<std::string, std::string> resolved;
  std::unordered_map<std::string, std::string> source;
  resolved.reserve(path_exp.size());

  for (const std::string & nid : path_exp) {
    // 1) Explicit tag from semantics YAML.
    auto it = seed_tags.find(nid);
    if (it != seed_tags.end()) {
      resolved[nid] = it->second;
      source  [nid] = "explicit";
      continue;
    }
    // 2) Proximity inheritance from co-located junction clusters
    //    (intersection / roundabout / crosswalk only).
    const auto * gn = g.node(nid);
    if (gn) {
      std::string best_region;
      double      best_d2 = kClusterRadiusM * kClusterRadiusM;
      for (const auto & [region, seeds] : seeds_by_region) {
        for (const auto * s : seeds) {
          const double dx = s->x - gn->x;
          const double dy = s->y - gn->y;
          const double d2 = dx * dx + dy * dy;
          if (d2 < best_d2) { best_d2 = d2; best_region = region; }
        }
      }
      if (!best_region.empty()) {
        resolved[nid] = best_region;
        source  [nid] = "co-located (" + std::to_string(std::sqrt(best_d2)) + " m)";
        continue;
      }
    }
    // 3) Default — normal.
    resolved[nid] = "normal";
    source  [nid] = "default";
  }

  // ---- Counters ----------------------------------------------------------
  std::unordered_map<std::string, int> per_region_count;
  std::unordered_map<std::string, int> inherited_count;   // inserted-and-tagged
  for (const std::string & nid : path_exp) {
    const std::string & r = resolved[nid];
    ++per_region_count[r];
    if (!original_set.count(nid) && r != "normal") ++inherited_count[r];
  }

  std::cerr << "per-region count (entire expanded mission):\n";
  for (const auto & [r, c] : per_region_count) std::cerr << "  " << r << " = " << c << "\n";
  std::cerr << "inserted nodes (NEW) given non-normal tag by inheritance:\n";
  if (inherited_count.empty()) std::cerr << "  (none — no inserted node fell into a propagatable cluster)\n";
  for (const auto & [r, c] : inherited_count) std::cerr << "  " << r << " = " << c << "\n";

  // ---- Write CSV report --------------------------------------------------
  {
    std::ofstream f(report_csv);
    f << "idx,node,x,y,region,source,is_inserted\n";
    f << std::fixed;
    for (std::size_t i = 0; i < path_exp.size(); ++i) {
      const std::string & nid = path_exp[i];
      const auto * gn = g.node(nid);
      const std::string & r = resolved[nid];
      const std::string & s = source  [nid];
      const bool inserted = !original_set.count(nid);
      f << i << "," << nid << ","
        << (gn ? gn->x : 0.0) << "," << (gn ? gn->y : 0.0) << ","
        << r << "," << s << "," << (inserted ? "1" : "0") << "\n";
    }
    std::cerr << "wrote " << report_csv << "\n";
  }

  // ---- Write competition_semantics_expanded.yaml -------------------------
  // Group by region. Per-region nodes list (explicit per-id, no ranges) so
  // downstream loaders are unambiguous.
  std::unordered_map<std::string, std::vector<std::string>> region_to_nodes;
  for (const std::string & nid : path_exp) {
    region_to_nodes[resolved[nid]].push_back(nid);
  }
  {
    std::ofstream f(semantics_out);
    f << "# =============================================================================\n"
      << "# competition_semantics_expanded.yaml — auto-generated by mission_semantics_inherit\n"
      << "# =============================================================================\n"
      << "# Each node in the expanded mission (" << path_exp.size()
      << " nodes) has an explicit region tag. No inserted node remains untagged.\n"
      << "#\n"
      << "# Inheritance rules:\n"
      << "#   1) seed tag from competition_semantics.yaml wins\n"
      << "#   2) co-located cluster inheritance for intersection / roundabout / crosswalk\n"
      << "#      (proximity threshold " << kClusterRadiusM << " m)\n"
      << "#   3) default = normal\n"
      << "# =============================================================================\n\n"
      << "default_region: \"normal\"\n\n"
      << "regions:\n";
    // Deterministic region ordering.
    std::vector<std::string> region_order = {
        "intersection", "roundabout", "crosswalk", "highway_entry",
        "highway_exit", "highway", "parking", "tunnel", "low_visibility",
        "no_lane", "normal"};
    for (const std::string & r : region_order) {
      auto it = region_to_nodes.find(r);
      if (it == region_to_nodes.end() || it->second.empty()) continue;
      f << "  " << r << ":\n"
        << "    nodes: [";
      bool first = true;
      for (const std::string & nid : it->second) {
        if (!first) f << ", ";
        f << "\"" << nid << "\"";
        first = false;
      }
      f << "]\n";
    }
    // Any region we forgot to put in region_order:
    for (auto & [r, nodes] : region_to_nodes) {
      if (std::find(region_order.begin(), region_order.end(), r) != region_order.end()) continue;
      f << "  " << r << ":\n    nodes: [";
      bool first = true;
      for (const std::string & nid : nodes) {
        if (!first) f << ", ";
        f << "\"" << nid << "\"";
        first = false;
      }
      f << "]\n";
    }
    std::cerr << "wrote " << semantics_out << "\n";
  }

  return 0;
}
