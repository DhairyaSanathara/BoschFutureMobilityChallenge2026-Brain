#include "global_path_planner_cpp/zone_map.hpp"

#include <sstream>

#include <yaml-cpp/yaml.h>

namespace global_path_planner_cpp
{

std::string ZoneMap::edge_key(const std::string & src, const std::string & tgt)
{
  return src + ">" + tgt;
}

bool ZoneMap::load(const std::string & yaml_path, std::string & error)
{
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);

    // ---- node_types -------------------------------------------------------
    if (root["node_types"] && root["node_types"].IsMap()) {
      for (const auto & it : root["node_types"]) {
        node_types[it.first.as<std::string>()] = it.second.as<std::string>("");
      }
    }
    // ---- semaphores -------------------------------------------------------
    if (root["semaphores"] && root["semaphores"].IsMap()) {
      for (const auto & it : root["semaphores"]) {
        semaphores[it.first.as<std::string>()] = it.second.as<std::string>("");
      }
    }
    // ---- highway_edges ----------------------------------------------------
    if (root["highway_edges"] && root["highway_edges"].IsSequence()) {
      for (const auto & e : root["highway_edges"]) {
        if (!e.IsMap()) continue;
        const auto src = e["source"].as<std::string>("");
        const auto tgt = e["target"].as<std::string>("");
        if (!src.empty() && !tgt.empty()) highway_edges.insert(edge_key(src, tgt));
      }
    }
    // ---- edge_speed_limits ------------------------------------------------
    if (root["edge_speed_limits"]) {
      const auto & esl = root["edge_speed_limits"];
      if (esl["default"]) default_edge_speed_limit = esl["default"].as<double>(default_edge_speed_limit);
      if (esl.IsSequence()) {
        for (const auto & e : esl) {
          if (!e.IsMap()) continue;
          const auto src = e["source"].as<std::string>("");
          const auto tgt = e["target"].as<std::string>("");
          const auto v   = e["speed_limit"].as<double>(default_edge_speed_limit);
          if (!src.empty() && !tgt.empty()) {
            edge_speed_overrides[edge_key(src, tgt)] = v;
          }
        }
      }
    }
    return true;
  } catch (const std::exception & e) {
    error = std::string("yaml-cpp: ") + e.what();
    return false;
  }
}

void ZoneMap::apply_speed_limits_to(Graph & /*g*/) const
{
  // Hook for future use. Graph's edge list is owned via adjacency map and
  // we'd need a mutator; today edge cost is set at insertion time and is
  // already `length / max(speed_limit, ε)` with a 1.0 m/s default. Patching
  // costs after the fact is complexity we don't need until a real zones.yaml
  // exists with non-default speeds. Implementing this is a 30-line follow-up
  // — left as a no-op stub to keep the contract obvious.
  //
  // When you DO need it, expose Graph::mutable_edges_from(id) and walk every
  // edge, replacing cost = length / max(override, default_edge_speed_limit).
}

const std::string * ZoneMap::node_type(const std::string & id) const
{
  auto it = node_types.find(id);
  return (it == node_types.end()) ? nullptr : &it->second;
}

const std::string * ZoneMap::semaphore_for(const std::string & id) const
{
  auto it = semaphores.find(id);
  return (it == semaphores.end()) ? nullptr : &it->second;
}

bool ZoneMap::edge_is_highway(const std::string & src,
                              const std::string & tgt) const
{
  return highway_edges.find(edge_key(src, tgt)) != highway_edges.end();
}

}  // namespace global_path_planner_cpp
