#ifndef OBJECT_DETECTION_CPP__CONFIG_LOADER_HPP_
#define OBJECT_DETECTION_CPP__CONFIG_LOADER_HPP_

#include <string>
#include <vector>

namespace object_detection_cpp
{

struct ModelConfig
{
  std::string engine_path;
  int input_width{640};
  int input_height{640};
};

struct RuntimeConfig
{
  int publish_rate{30};
  int warmup_runs{10};
  double max_log_rate_hz{1.0};
  double watchdog_timeout_s{0.5};
};

struct ThresholdConfig
{
  float conf{0.45f};
  float nms{0.45f};
};

struct TopicConfig
{
  std::string subscribe{"/camera/camera/color/image_raw"};
  std::string publish{"/perception/objects"};
  std::string depth{"/camera/camera/depth/image_rect_raw"};
  std::string camera_info{"/camera/camera/color/camera_info"};
};

struct DetectionConfig
{
  ModelConfig model;
  RuntimeConfig runtime;
  ThresholdConfig thresholds;
  std::vector<std::string> classes;
  TopicConfig topics;

  size_t num_classes() const { return classes.size(); }
};

DetectionConfig load_config(const std::string & path);

}  // namespace object_detection_cpp

#endif  // OBJECT_DETECTION_CPP__CONFIG_LOADER_HPP_
