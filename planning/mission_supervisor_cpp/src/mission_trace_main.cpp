// Standalone audit tool — for every node in the expanded mission, write a
// CSV row (idx, node, x, y, region, speed_limit, behavior, nav_flag)
// representing exactly what the runtime mission_semantics_node would publish
// when the vehicle is nearest to that node. This is a dry-run trace operators
// can sanity-check before race day, no ROS subscriptions required.
//
// Usage:
//   mission_trace <graphml>
//                 <competition_mission_expanded.yaml>
//                 <competition_semantics_expanded.yaml>
//                 <competition_speed_profiles.yaml>
//                 <competition_behavior_overrides.yaml>
//                 <out_trace.csv>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "mission_supervisor_cpp/mission_graph.hpp"

using mission_supervisor_cpp::MissionGraph;

namespace {

std::vector<std::string> load_path_(const std::string & path)
{
  std::vector<std::string> out;
  YAML::Node y = YAML::LoadFile(path);
  for (const auto & n : y["mission"]["path"]) out.push_back(n.as<std::string>());
  return out;
}

std::unordered_map<std::string, std::string> load_node_regions_(const std::string & path)
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
        int lo = r[0].as<int>(), hi = r[1].as<int>();
        if (lo > hi) std::swap(lo, hi);
        for (int i = lo; i <= hi; ++i) out[std::to_string(i)] = region;
      }
    }
  }
  return out;
}

std::unordered_map<std::string, double> load_speeds_(const std::string & path)
{
  std::unordered_map<std::string, double> out;
  YAML::Node y = YAML::LoadFile(path);
  auto sp = y["speed_profiles"];
  if (!sp) return out;
  for (auto it = sp.begin(); it != sp.end(); ++it) {
    out[it->first.as<std::string>()] = it->second.as<double>();
  }
  return out;
}

struct Disp {
  std::string maneuver{"LANE_KEEP"};
  std::string nav_flag;
};
std::unordered_map<std::string, Disp> load_overrides_(const std::string & path)
{
  std::unordered_map<std::string, Disp> out;
  YAML::Node y = YAML::LoadFile(path);
  auto ov = y["overrides"];
  if (!ov) return out;
  for (auto it = ov.begin(); it != ov.end(); ++it) {
    const std::string region = it->first.as<std::string>();
    Disp d;
    if (it->second["maneuver"]) d.maneuver = it->second["maneuver"].as<std::string>();
    if (it->second["nav_flag"]) d.nav_flag = it->second["nav_flag"].as<std::string>();
    out[region] = d;
  }
  return out;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 7) {
    std::cerr << "usage: " << argv[0]
              << " <graphml> <mission_expanded.yaml> <semantics_expanded.yaml> "
              << "<speeds.yaml> <overrides.yaml> <out.csv>\n";
    return 64;
  }
  const std::string graphml      = argv[1];
  const std::string mission_yaml = argv[2];
  const std::string semantics_y  = argv[3];
  const std::string speeds_y     = argv[4];
  const std::string overrides_y  = argv[5];
  const std::string out_csv      = argv[6];

  MissionGraph g;
  std::string err;
  if (!g.load(graphml, err)) { std::cerr << "FATAL: " << err << "\n"; return 64; }

  auto path      = load_path_(mission_yaml);
  auto regions   = load_node_regions_(semantics_y);
  auto speeds    = load_speeds_(speeds_y);
  auto overrides = load_overrides_(overrides_y);

  auto region_for = [&](const std::string & nid) -> std::string {
    auto it = regions.find(nid);
    return it == regions.end() ? "normal" : it->second;
  };
  auto speed_for = [&](const std::string & region) -> double {
    auto it = speeds.find(region);
    return it == speeds.end() ? 0.25 : it->second;
  };
  auto disp_for = [&](const std::string & region) -> Disp {
    auto it = overrides.find(region);
    return it == overrides.end() ? Disp{} : it->second;
  };

  std::ofstream f(out_csv);
  f << "idx,node,x,y,region,speed_limit,behavior,nav_flag\n";
  f << std::fixed;

  // Region-transition log.
  std::vector<std::tuple<std::size_t, std::string, std::string, std::string>> transitions;
  std::string prev_region;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const std::string & nid = path[i];
    const auto * gn = g.node(nid);
    const std::string region = region_for(nid);
    const double v = speed_for(region);
    const Disp d   = disp_for(region);

    f << i << "," << nid << ","
      << std::setprecision(3) << (gn ? gn->x : 0.0) << ","
      << (gn ? gn->y : 0.0) << ","
      << region << ","
      << std::setprecision(2) << v << ","
      << d.maneuver << ","
      << d.nav_flag << "\n";

    if (region != prev_region) {
      transitions.emplace_back(i, prev_region.empty() ? std::string("(start)") : prev_region,
                               region, nid);
      prev_region = region;
    }
  }
  std::cerr << "wrote " << out_csv << " (" << path.size() << " rows)\n";

  // Region transition summary.
  std::cerr << "\nregion transitions:\n";
  std::cerr << std::left << std::setw(6) << "idx"
            << std::setw(8) << "node"
            << "from → to\n";
  for (const auto & [i, from, to, nid] : transitions) {
    std::cerr << std::setw(6) << i
              << std::setw(8) << nid
              << from << " → " << to << "\n";
  }
  return 0;
}
