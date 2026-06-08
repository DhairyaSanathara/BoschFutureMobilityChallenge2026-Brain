#ifndef OBJECT_DETECTION_CPP__PREPROCESS_CUDA_HPP_
#define OBJECT_DETECTION_CPP__PREPROCESS_CUDA_HPP_

#include <cstdint>
#include <cuda_runtime.h>

namespace object_detection_cpp
{

// Fused CUDA preprocess matching the original Python pipeline EXACTLY:
//   1. Bilinear resize to (dst_h x dst_w) using OpenCV INTER_LINEAR pixel-center
//      convention: src = (dst + 0.5) * scale - 0.5, BORDER_REPLICATE clamp.
//   2. Channel swap when input is BGR (OpenCV cv2.cvtColor(BGR2RGB)).
//      No swap when input is already RGB (RealSense default rgb8).
//   3. Normalize divide by 255.0f.
//   4. Scatter into NCHW float32 plane layout consumed by TensorRT.
//
// All operations are float (numerically equivalent to the Python pipeline
// up to expected floating-point error).
//
// Arguments
// ---------
//   src         : device pointer to packed HxWx3 uint8 image (row-stride = src_step)
//   src_h/w     : source height, width (pixels)
//   src_step    : source row stride in bytes
//   src_is_bgr  : true for bgr8 encoding, false for rgb8
//   dst         : device pointer to NCHW float32 tensor (1*3*dst_h*dst_w floats)
//   dst_h/w    : destination height, width
//   stream      : CUDA stream on which to enqueue the kernel
//
// Returns the cudaError_t result of the kernel launch (cudaPeekAtLastError).
cudaError_t launch_preprocess(
  const uint8_t * src,
  int src_h, int src_w, int src_step,
  bool src_is_bgr,
  float * dst,
  int dst_h, int dst_w,
  cudaStream_t stream);

}  // namespace object_detection_cpp

#endif  // OBJECT_DETECTION_CPP__PREPROCESS_CUDA_HPP_
