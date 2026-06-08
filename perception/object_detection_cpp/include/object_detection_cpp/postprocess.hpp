#ifndef OBJECT_DETECTION_CPP__POSTPROCESS_HPP_
#define OBJECT_DETECTION_CPP__POSTPROCESS_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace object_detection_cpp
{

struct Detection
{
  std::array<float, 4> bbox;   // x1, y1, x2, y2 in source-image pixel coords
  float confidence;
  int class_id;
};

// YOLOv8 head decoder + per-class NMS, matching the semantics of the original
// postprocess.py (cxcywh -> xyxy, conf gate, batched per-class greedy NMS,
// rescale from model coords to source-image coords, clamp).
//
// raw_output is laid out as (1, 4 + num_classes, N) row-major — i.e. the same
// layout produced by Ultralytics YOLOv8 ONNX export. We index it as
// [c * N + i] for predictions[i, c].
//
// out is cleared and populated; the function returns the number of detections.
size_t decode_yolov8(
  const float * raw_output,
  int num_classes,
  int num_predictions,
  float conf_threshold,
  float nms_threshold,
  int src_h, int src_w,
  int input_h, int input_w,
  std::vector<Detection> & out);

}  // namespace object_detection_cpp

#endif  // OBJECT_DETECTION_CPP__POSTPROCESS_HPP_
