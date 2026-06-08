#include "object_detection_cpp/config_loader.hpp"

#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace object_detection_cpp
{

namespace fs = std::filesystem;

DetectionConfig load_config(const std::string & path)
{
  if (!fs::exists(path)) {
    throw std::runtime_error("config file not found: " + path);
  }

  YAML::Node root = YAML::LoadFile(path);

  if (!root["model"] || !root["runtime"] || !root["thresholds"] ||
      !root["classes"] || !root["topics"])
  {
    throw std::runtime_error("config missing required top-level section");
  }

  DetectionConfig cfg;

  cfg.model.engine_path  = root["model"]["engine_path"].as<std::string>();
  cfg.model.input_width  = root["model"]["input_width"].as<int>(640);
  cfg.model.input_height = root["model"]["input_height"].as<int>(640);

  cfg.runtime.publish_rate       = root["runtime"]["publish_rate"].as<int>(30);
  cfg.runtime.warmup_runs        = root["runtime"]["warmup_runs"].as<int>(10);
  cfg.runtime.max_log_rate_hz    = root["runtime"]["max_log_rate_hz"].as<double>(1.0);
  cfg.runtime.watchdog_timeout_s = root["runtime"]["watchdog_timeout_s"].as<double>(0.5);

  cfg.thresholds.conf = root["thresholds"]["conf"].as<float>(0.45f);
  cfg.thresholds.nms  = root["thresholds"]["nms"].as<float>(0.45f);

  for (const auto & c : root["classes"]) {
    cfg.classes.push_back(c.as<std::string>());
  }
  if (cfg.classes.empty()) {
    throw std::runtime_error("config.classes must be a non-empty list");
  }

  cfg.topics.subscribe   = root["topics"]["subscribe"].as<std::string>(cfg.topics.subscribe);
  cfg.topics.publish     = root["topics"]["publish"].as<std::string>(cfg.topics.publish);
  cfg.topics.depth       = root["topics"]["depth"].as<std::string>(cfg.topics.depth);
  cfg.topics.camera_info = root["topics"]["camera_info"].as<std::string>(cfg.topics.camera_info);

  if (cfg.thresholds.conf <= 0.f || cfg.thresholds.conf >= 1.f) {
    throw std::runtime_error("thresholds.conf must be in (0,1)");
  }
  if (cfg.runtime.publish_rate <= 0) {
    throw std::runtime_error("runtime.publish_rate must be positive");
  }
  // engine_path may legitimately be empty in the YAML and overridden by
  // the engine_path node parameter; the node enforces non-emptiness after
  // applying that override.

  return cfg;
}

}  // namespace object_detection_cpp
