#include "global_path_planner_cpp/planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_path_planner_cpp/graphml_parser.hpp"

namespace global_path_planner_cpp
{

GlobalPathPlannerNode::GlobalPathPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("global_path_planner_node", options)
{
  // -------- Parameters --------
  graphml_path_       = declare_parameter<std::string>("graphml_path", graphml_path_);
  zones_yaml_path_    = declare_parameter<std::string>("zones_yaml_path", zones_yaml_path_);
  start_node_         = declare_parameter<std::string>("start_node",  start_node_);
  goal_node_          = declare_parameter<std::string>("goal_node",    goal_node_);
  frame_id_           = declare_parameter<std::string>("frame_id",     frame_id_);
  replan_deviation_m_ = declare_parameter<double>("replan_deviation_m", replan_deviation_m_);
  republish_period_s_ = declare_parameter<double>("republish_period_s", republish_period_s_);
  disable_pub_        = declare_parameter<bool>("disable_pub", disable_pub_);
  replan_consecutive_    = declare_parameter<int>   ("replan_consecutive_samples", replan_consecutive_);
  replan_min_speed_mps_  = declare_parameter<double>("replan_min_speed_mps",       replan_min_speed_mps_);
  replan_min_interval_s_ = declare_parameter<double>("replan_min_interval_s",      replan_min_interval_s_);
  const std::string algo_str =
    declare_parameter<std::string>("algorithm", std::string("astar"));
  algo_ = (algo_str == "dijkstra") ? Algo::Dijkstra : Algo::AStar;
  if (disable_pub_) {
    RCLCPP_INFO(get_logger(),
      "global_path_planner: disable_pub=true — /planning/global_path and "
      "/navigation/flags pubs are SUPPRESSED (mission_supervisor owns those).");
  }

  // -------- Zone side-car (Phase Final) ----------------------------------
  if (!zones_yaml_path_.empty()) {
    std::string zerr;
    if (zones_.load(zones_yaml_path_, zerr)) {
      RCLCPP_INFO(get_logger(),
        "zones.yaml loaded from '%s' (node_types=%zu, semaphores=%zu, "
        "highway_edges=%zu)",
        zones_yaml_path_.c_str(),
        zones_.node_types.size(), zones_.semaphores.size(),
        zones_.highway_edges.size());
    } else {
      RCLCPP_WARN(get_logger(),
        "zones.yaml load FAILED ('%s'): %s — proceeding with empty zone metadata",
        zones_yaml_path_.c_str(), zerr.c_str());
    }
  } else {
    RCLCPP_INFO(get_logger(),
      "zones.yaml not configured — nav_flags will be empty unless re-loaded later");
  }

  // -------- Map loading --------
  if (graphml_path_.empty()) {
    RCLCPP_WARN(get_logger(),
      "global_path_planner: graphml_path parameter is empty — node will sit "
      "idle until a non-empty path is provided. Planning disabled.");
  } else {
    std::string msg;
    graph_ready_ = load_graphml(graphml_path_, graph_, msg);
    if (graph_ready_) {
      RCLCPP_INFO(get_logger(), "graphml loaded from '%s' (%s)",
                  graphml_path_.c_str(), msg.c_str());
      RCLCPP_INFO(get_logger(),
                  "graph: %zu nodes, %zu edges, algo=%s",
                  graph_.n_nodes(), graph_.n_edges(),
                  algo_ == Algo::AStar ? "A*" : "Dijkstra");
    } else {
      RCLCPP_ERROR(get_logger(),
        "graphml load FAILED from '%s' — %s. Planning disabled until reload.",
        graphml_path_.c_str(), msg.c_str());
    }
  }

  // -------- ROS interfaces --------
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  auto path_qos  = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
  auto dbg_qos   = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  sub_state_ = create_subscription<bfmc_msgs::msg::VehicleState>(
    "/vehicle/state", state_qos,
    std::bind(&GlobalPathPlannerNode::on_vehicle_state, this, std::placeholders::_1));

  // Optional runtime override of the goal node — write a node id on this
  // topic to re-plan to a new destination without re-launching.
  sub_goal_ = create_subscription<std_msgs::msg::String>(
    "/planning/goal_node", state_qos,
    std::bind(&GlobalPathPlannerNode::on_goal_command, this, std::placeholders::_1));

  pub_path_     = create_publisher<nav_msgs::msg::Path>(
    "/planning/global_path", path_qos);
  pub_nav_flag_ = create_publisher<std_msgs::msg::String>(
    "/navigation/flags", state_qos);
  pub_dbg_node_ = create_publisher<std_msgs::msg::String>(
    "/planning/debug/current_node", dbg_qos);
  pub_dbg_goal_ = create_publisher<std_msgs::msg::String>(
    "/planning/debug/current_goal", dbg_qos);
  pub_dbg_dev_  = create_publisher<std_msgs::msg::Float64>(
    "/planning/debug/path_deviation", dbg_qos);

  republish_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(republish_period_s_ * 1000.0)),
    std::bind(&GlobalPathPlannerNode::republish_timer_tick, this));

  RCLCPP_INFO(get_logger(),
    "global_path_planner: start='%s' goal='%s' algo=%s replan_dev=%.2fm republish=%.1fs",
    start_node_.c_str(), goal_node_.c_str(),
    algo_ == Algo::AStar ? "A*" : "Dijkstra",
    replan_deviation_m_, republish_period_s_);

  // ----- Fallback startup: plan immediately if both start + goal supplied --
  // Lets the planner emit /planning/global_path without /vehicle/state, which
  // is critical when localization is down or for bench testing.
  if (graph_ready_ && !start_node_.empty() && !goal_node_.empty()) {
    if (!graph_.node(start_node_)) {
      RCLCPP_WARN(get_logger(),
        "fallback start_node '%s' not in graph — ignoring", start_node_.c_str());
    } else {
      current_nearest_id_ = start_node_;
      const bool ok = plan_to_goal_();
      RCLCPP_INFO(get_logger(),
        "fallback initial plan: %s (start='%s' goal='%s')",
        ok ? "OK" : "FAILED", start_node_.c_str(), goal_node_.c_str());

      // Also fire the one-shot nav_flag for the start node, since we won't
      // get an on_vehicle_state callback to drive the nearest-node delta
      // logic in fallback mode.
      std::string flag;
      if (const auto * t = zones_.node_type(start_node_)) flag = *t;
      if (!flag.empty()) {
        last_nav_flag_ = flag;
        if (!disable_pub_) {
          std_msgs::msg::String f; f.data = flag;
          pub_nav_flag_->publish(f);
        }
      }
      std_msgs::msg::String nm; nm.data = start_node_; pub_dbg_node_->publish(nm);
    }
  }
}

void GlobalPathPlannerNode::on_vehicle_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y)) return;
  last_x_ = msg->pose.position.x;
  last_y_ = msg->pose.position.y;
  last_speed_mps_ = std::isfinite(msg->speed) ? msg->speed : 0.0f;
  have_state_.store(true, std::memory_order_release);

  if (!graph_ready_ || goal_node_.empty()) return;

  // Nearest-node tracking (cheap O(N) — see Graph::nearest_node).
  const std::string nn = graph_.nearest_node(last_x_, last_y_);
  if (!nn.empty() && nn != current_nearest_id_) {
    current_nearest_id_ = nn;
    std_msgs::msg::String m; m.data = nn;
    pub_dbg_node_->publish(m);

    // ------- Phase Final: publish /navigation/flags from zones.yaml ------
    // Two sources of truth, merged with a fixed priority:
    //   1. nearest-node type (intersection / parking / roundabout / trafficlight /
    //      highway_entry / highway_exit / crosswalk)
    //   2. current-edge highway membership (overrides if type is plain "road")
    // We dedup so the same flag is not re-published every state callback.
    std::string flag;
    if (const auto * t = zones_.node_type(current_nearest_id_)) flag = *t;
    if (flag.empty() && !current_path_.empty()) {
      // Walk the cached path forward one step from the nearest node to find
      // the current edge; check for highway membership.
      for (std::size_t i = 0; i + 1 < current_path_.size(); ++i) {
        if (current_path_[i] == current_nearest_id_) {
          if (zones_.edge_is_highway(current_path_[i], current_path_[i + 1])) {
            flag = "highway";
          }
          break;
        }
      }
    }
    if (flag != last_nav_flag_) {
      last_nav_flag_ = flag;
      if (!disable_pub_) {
        std_msgs::msg::String f; f.data = flag;
        pub_nav_flag_->publish(f);
      }
      RCLCPP_INFO(get_logger(),
        "nav_flag → '%s' (nearest node '%s')%s",
        flag.c_str(), current_nearest_id_.c_str(),
        disable_pub_ ? " [SUPPRESSED — mission_supervisor owns this]" : "");
    }
  }

  // Replan triggers — split into "catastrophic" (one-shot recovery, always
  // fire) and "deviation" (subject to the replan storm guard).
  //   catastrophic: no current path yet, OR cached start ≠ current nearest
  //   deviation:    lateral deviation exceeds threshold for N consecutive
  //                 samples, AND vehicle is moving, AND last replan ≥ Ns ago.
  bool need_replan = current_path_.empty();
  if (!need_replan && current_path_.front() != current_nearest_id_) need_replan = true;

  if (!need_replan) {
    const double dev = distance_to_path(graph_, last_x_, last_y_, current_path_);
    std_msgs::msg::Float64 d; d.data = dev; pub_dbg_dev_->publish(d);

    const bool over = std::isfinite(dev) && dev > replan_deviation_m_;
    if (over) {
      dev_over_count_ = std::min(dev_over_count_ + 1, replan_consecutive_ + 1);
    } else {
      dev_over_count_ = 0;
    }

    const bool consec_ok  = dev_over_count_ >= replan_consecutive_;
    const bool moving     = std::abs(last_speed_mps_) > replan_min_speed_mps_;
    const double since_s  = (last_replan_time_.nanoseconds() == 0)
                            ? std::numeric_limits<double>::infinity()
                            : (this->now() - last_replan_time_).seconds();
    const bool interval_ok = since_s >= replan_min_interval_s_;

    if (over) {
      RCLCPP_DEBUG(get_logger(),
        "deviation %.2fm > %.2fm  consec=%d/%d  moving=%d  since_replan=%.2fs",
        dev, replan_deviation_m_, dev_over_count_, replan_consecutive_,
        moving ? 1 : 0, since_s);
    }

    if (consec_ok && moving && interval_ok) {
      RCLCPP_INFO(get_logger(),
        "global_path_planner: deviation %.2fm > %.2fm sustained for %d samples "
        "@ v=%.2f m/s, %.2fs since last replan — replanning from '%s'",
        dev, replan_deviation_m_, dev_over_count_,
        static_cast<double>(last_speed_mps_), since_s,
        current_nearest_id_.c_str());
      need_replan = true;
    }
  }

  if (need_replan) {
    if (plan_to_goal_()) {
      last_replan_time_ = this->now();
      dev_over_count_   = 0;
    }
  }
}

void GlobalPathPlannerNode::on_goal_command(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data.empty() || msg->data == goal_node_) return;
  goal_node_ = msg->data;
  RCLCPP_INFO(get_logger(), "global_path_planner: goal updated → '%s'", goal_node_.c_str());
  if (have_state_.load(std::memory_order_acquire)) plan_to_goal_();
}

bool GlobalPathPlannerNode::plan_to_goal_()
{
  if (!graph_ready_) return false;
  if (current_nearest_id_.empty()) {
    current_nearest_id_ = graph_.nearest_node(last_x_, last_y_);
    if (current_nearest_id_.empty()) return false;
  }
  if (!graph_.node(goal_node_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "goal node '%s' not present in graph — skipping plan",
      goal_node_.c_str());
    return false;
  }

  std::vector<std::string> path;
  double cost = 0.0;
  const bool ok = plan(graph_, current_nearest_id_, goal_node_, algo_, path, cost);
  if (!ok) {
    RCLCPP_WARN(get_logger(),
      "no path from '%s' to '%s'", current_nearest_id_.c_str(), goal_node_.c_str());
    return false;
  }

  current_path_ = std::move(path);
  RCLCPP_INFO(get_logger(),
    "global_path: %zu nodes, cost %.2f (s @ unit-speed-equivalent)",
    current_path_.size(), cost);

  std_msgs::msg::String g; g.data = goal_node_; pub_dbg_goal_->publish(g);
  publish_current_path_();
  return true;
}

void GlobalPathPlannerNode::publish_current_path_()
{
  if (current_path_.empty()) return;

  nav_msgs::msg::Path msg;
  msg.header.stamp    = this->now();
  msg.header.frame_id = frame_id_;
  msg.poses.reserve(current_path_.size());

  // For each node we emit a PoseStamped at (x, y, 0) with orientation pointing
  // toward the NEXT node (last node points along the previous direction).
  for (std::size_t i = 0; i < current_path_.size(); ++i) {
    const auto * n = graph_.node(current_path_[i]);
    if (!n) continue;
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg.header;
    ps.pose.position.x = n->x;
    ps.pose.position.y = n->y;
    ps.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i + 1 < current_path_.size()) {
      const auto * nn = graph_.node(current_path_[i + 1]);
      if (nn) yaw = std::atan2(nn->y - n->y, nn->x - n->x);
    } else if (!msg.poses.empty()) {
      // copy heading from previous pose for the terminal node
      const auto & prev = msg.poses.back().pose.position;
      yaw = std::atan2(n->y - prev.y, n->x - prev.x);
    }
    const double hz = std::sin(yaw * 0.5);
    const double hw = std::cos(yaw * 0.5);
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = hz;
    ps.pose.orientation.w = hw;
    msg.poses.push_back(std::move(ps));
  }
  if (!disable_pub_) pub_path_->publish(msg);
}

void GlobalPathPlannerNode::republish_timer_tick()
{
  // Periodic re-emission so a late-arriving subscriber (e.g. behavior_planner
  // started after this node) can sync up without a service round-trip. Also
  // re-emits the latched nav_flag and current_goal because their publishers
  // are RELIABLE-not-transient-local — without this, late subscribers miss
  // the one-shot we published on configure.
  if (!current_path_.empty()) publish_current_path_();

  if (!last_nav_flag_.empty() && pub_nav_flag_ && !disable_pub_) {
    std_msgs::msg::String m; m.data = last_nav_flag_;
    pub_nav_flag_->publish(m);
  }
  if (!goal_node_.empty() && pub_dbg_goal_) {
    std_msgs::msg::String m; m.data = goal_node_;
    pub_dbg_goal_->publish(m);
  }
  if (!current_nearest_id_.empty() && pub_dbg_node_) {
    std_msgs::msg::String m; m.data = current_nearest_id_;
    pub_dbg_node_->publish(m);
  }
}

}  // namespace global_path_planner_cpp
