#ifndef GLOBAL_PATH_PLANNER_CPP__GRAPHML_PARSER_HPP_
#define GLOBAL_PATH_PLANNER_CPP__GRAPHML_PARSER_HPP_

#include <string>

#include "global_path_planner_cpp/graph.hpp"

namespace global_path_planner_cpp
{

// Loads a BFMC competition GraphML file into `graph`. Returns true on success;
// on failure `error` is populated with a one-line reason and `graph` may be
// partially populated (caller should discard).
//
// The parser is intentionally permissive about which attributes are present.
// Required: node `id` + `x` + `y` (the planner cannot run without geometry).
// Optional with defaults: node `type`; edge `speed_limit`, `road_type`,
// `priority`, `lane_direction`, `traffic_light_id`. Unknown <data key="..."/>
// entries are silently ignored — future map extensions don't break us.
bool load_graphml(const std::string & path, Graph & graph, std::string & error);

}  // namespace global_path_planner_cpp

#endif  // GLOBAL_PATH_PLANNER_CPP__GRAPHML_PARSER_HPP_
