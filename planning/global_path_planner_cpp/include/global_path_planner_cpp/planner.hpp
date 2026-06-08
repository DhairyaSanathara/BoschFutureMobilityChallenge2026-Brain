#ifndef GLOBAL_PATH_PLANNER_CPP__PLANNER_HPP_
#define GLOBAL_PATH_PLANNER_CPP__PLANNER_HPP_

#include <string>
#include <vector>

#include "global_path_planner_cpp/graph.hpp"

namespace global_path_planner_cpp
{

enum class Algo : std::uint8_t { Dijkstra = 0, AStar = 1 };

// Standard shortest-path search on a Graph. Returns true if a path was found;
// `out_path` is populated as a sequence of node ids from start to goal (both
// included). `out_cost` is the total edge-cost sum (travel-time minimisation).
//
// Both algorithms operate on `GraphEdge::cost`. A* uses Euclidean distance
// between nodes as the heuristic (admissible because cost = length/speed,
// and length itself is Euclidean — heuristic ≤ true remaining cost when
// speed_limit ≥ 1 m/s, which holds for every BFMC edge).
bool plan(const Graph & graph,
          const std::string & start,
          const std::string & goal,
          Algo algo,
          std::vector<std::string> & out_path,
          double & out_cost);

// Lateral (perpendicular) deviation of a point from a polyline traced through
// the given node sequence. Returns +∞ if the path has fewer than two nodes
// or any referenced node is missing. Used by the planner_node to decide
// whether to replan.
double distance_to_path(const Graph & graph,
                        double x, double y,
                        const std::vector<std::string> & path);

}  // namespace global_path_planner_cpp

#endif  // GLOBAL_PATH_PLANNER_CPP__PLANNER_HPP_
