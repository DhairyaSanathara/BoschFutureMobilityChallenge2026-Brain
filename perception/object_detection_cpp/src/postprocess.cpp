#include "object_detection_cpp/postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace object_detection_cpp
{

namespace
{

struct DecodedBox
{
  float x1, y1, x2, y2;
  float score;
  int   cls;
};

inline float iou_xyxy(const DecodedBox & a, const DecodedBox & b)
{
  const float xx1 = std::max(a.x1, b.x1);
  const float yy1 = std::max(a.y1, b.y1);
  const float xx2 = std::min(a.x2, b.x2);
  const float yy2 = std::min(a.y2, b.y2);
  const float w = std::max(0.0f, xx2 - xx1);
  const float h = std::max(0.0f, yy2 - yy1);
  const float inter = w * h;
  const float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
  const float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
  return inter / (area_a + area_b - inter + 1e-6f);
}

}  // namespace

size_t decode_yolov8(
  const float * raw,
  int num_classes,
  int N,
  float conf_threshold,
  float nms_threshold,
  int src_h, int src_w,
  int input_h, int input_w,
  std::vector<Detection> & out)
{
  out.clear();
  if (raw == nullptr || N <= 0 || num_classes <= 0) {
    return 0;
  }

  // YOLOv8 ONNX export layout: (1, 4 + nc, N) row-major.
  // Channel c, prediction i lives at raw[c * N + i].
  const int total_ch = 4 + num_classes;
  (void)total_ch;

  // ---------------------------------------------------------------------------
  // Confidence gate + cxcywh -> xyxy. Reserve once.
  // ---------------------------------------------------------------------------
  std::vector<DecodedBox> filtered;
  filtered.reserve(256);

  for (int i = 0; i < N; ++i) {
    // Find best class for this prediction.
    int best_cls = 0;
    float best_score = raw[4 * N + i];  // first class channel
    for (int c = 1; c < num_classes; ++c) {
      const float s = raw[(4 + c) * N + i];
      if (s > best_score) {
        best_score = s;
        best_cls = c;
      }
    }

    if (best_score < conf_threshold) {
      continue;
    }

    const float cx = raw[0 * N + i];
    const float cy = raw[1 * N + i];
    const float w  = raw[2 * N + i];
    const float h  = raw[3 * N + i];
    const float hw = w * 0.5f;
    const float hh = h * 0.5f;

    DecodedBox b;
    b.x1 = cx - hw;
    b.y1 = cy - hh;
    b.x2 = cx + hw;
    b.y2 = cy + hh;
    b.score = best_score;
    b.cls = best_cls;
    filtered.push_back(b);
  }

  if (filtered.empty()) {
    return 0;
  }

  // ---------------------------------------------------------------------------
  // Per-class greedy NMS: sort by score desc, then drop overlaps within same class.
  // Class offset trick is unnecessary because we filter same-class only.
  // ---------------------------------------------------------------------------
  std::vector<int> order(filtered.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int a, int b) { return filtered[a].score > filtered[b].score; });

  std::vector<uint8_t> suppressed(filtered.size(), 0);

  // ---------------------------------------------------------------------------
  // Rescale model coords -> source-image coords (no letterbox; YOLO was fed a
  // direct resize, identical to the original Python pipeline).
  // ---------------------------------------------------------------------------
  const float sx = static_cast<float>(src_w) / static_cast<float>(input_w);
  const float sy = static_cast<float>(src_h) / static_cast<float>(input_h);

  for (size_t i = 0; i < order.size(); ++i) {
    const int idx_i = order[i];
    if (suppressed[idx_i]) continue;

    const DecodedBox & bi = filtered[idx_i];

    Detection det;
    det.bbox[0] = std::clamp(bi.x1 * sx, 0.0f, static_cast<float>(src_w));
    det.bbox[1] = std::clamp(bi.y1 * sy, 0.0f, static_cast<float>(src_h));
    det.bbox[2] = std::clamp(bi.x2 * sx, 0.0f, static_cast<float>(src_w));
    det.bbox[3] = std::clamp(bi.y2 * sy, 0.0f, static_cast<float>(src_h));
    det.confidence = bi.score;
    det.class_id = bi.cls;
    out.push_back(det);

    for (size_t j = i + 1; j < order.size(); ++j) {
      const int idx_j = order[j];
      if (suppressed[idx_j]) continue;
      const DecodedBox & bj = filtered[idx_j];
      if (bj.cls != bi.cls) continue;
      if (iou_xyxy(bi, bj) > nms_threshold) {
        suppressed[idx_j] = 1;
      }
    }
  }

  return out.size();
}

}  // namespace object_detection_cpp
