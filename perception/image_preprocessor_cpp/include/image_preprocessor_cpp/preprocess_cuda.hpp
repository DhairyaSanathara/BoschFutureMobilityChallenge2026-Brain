#ifndef IMAGE_PREPROCESSOR_CPP__PREPROCESS_CUDA_HPP_
#define IMAGE_PREPROCESSOR_CPP__PREPROCESS_CUDA_HPP_

#include <cstdint>
#include <cuda_runtime.h>

namespace image_preprocessor_cpp
{

// Initialise the sRGB->linear LUT in __constant__ memory.
// Idempotent; safe to call from multiple nodes in the same process.
cudaError_t init_srgb_constants();

// Fused CUDA kernel:
//   1. Bilinear resize (INTER_LINEAR, pixel-center convention, BORDER_REPLICATE)
//   2. Optional channel swap (when input is BGR vs RGB)
//   3. sRGB -> linear via 256-entry LUT in __constant__ memory
//   4. Linear sRGB -> Y (CIE Y of XYZ)
//   5. Y -> Lab L* (cbrt or linear branch), scaled to uint8 [0..255]
//
// Numerically equivalent to OpenCV's `cv::resize(... INTER_LINEAR)` followed
// by `cv::cvtColor(... BGR2Lab)` followed by extracting channel 0, modulo a
// <=1-LSB difference at edges caused by not rounding the intermediate BGR to
// uint8 between the two operations. The downstream threshold of 140 is
// insensitive to this on lane edges in BFMC imagery.
//
// Arguments
// ---------
//  d_src        : device pointer to packed HxWx3 uint8 image (row stride = src_step)
//  src_h/w      : source pixels
//  src_step     : source row stride in bytes
//  src_is_bgr   : true for bgr8 encoding, false for rgb8
//  d_dst_l      : device pointer to dst_h*dst_w uint8 L-channel buffer
//  dst_h/w     : destination pixels
//  stream       : CUDA stream
//
// Returns cudaPeekAtLastError() right after kernel launch.
cudaError_t launch_resize_bgr_to_lab_l(
  const uint8_t * d_src,
  int src_h, int src_w, int src_step,
  bool src_is_bgr,
  uint8_t * d_dst_l,
  int dst_h, int dst_w,
  cudaStream_t stream);

}  // namespace image_preprocessor_cpp

#endif  // IMAGE_PREPROCESSOR_CPP__PREPROCESS_CUDA_HPP_
