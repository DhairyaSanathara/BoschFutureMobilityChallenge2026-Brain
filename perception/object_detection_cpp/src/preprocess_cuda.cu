// =============================================================================
// preprocess_cuda.cu
// Fused YOLO preprocess kernel: resize (OpenCV INTER_LINEAR pixel-center
// convention, BORDER_REPLICATE) -> optional BGR<->RGB swap -> /255 -> NCHW
// scatter into the TensorRT input tensor.
//
// Numerically equivalent to the original CPU pipeline:
//   resize(BGR, dst, INTER_LINEAR) -> cvtColor(BGR2RGB) -> /255 -> CHW
// (modulo float vs. fixed-point bilinear, which the model is robust to.)
// =============================================================================

#include "object_detection_cpp/preprocess_cuda.hpp"

namespace object_detection_cpp
{

namespace
{

__device__ __forceinline__ int clamp_i(int v, int lo, int hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

template <bool SrcIsBgr>
__global__ void preprocess_kernel(
  const uint8_t * __restrict__ src,
  int src_h, int src_w, int src_step,
  float * __restrict__ dst,
  int dst_h, int dst_w,
  float scale_x, float scale_y)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= dst_w || y >= dst_h) {
    return;
  }

  // OpenCV INTER_LINEAR centered-pixel mapping.
  float fx = (static_cast<float>(x) + 0.5f) * scale_x - 0.5f;
  float fy = (static_cast<float>(y) + 0.5f) * scale_y - 0.5f;

  int x0 = static_cast<int>(floorf(fx));
  int y0 = static_cast<int>(floorf(fy));
  int x1 = x0 + 1;
  int y1 = y0 + 1;

  const float dx = fx - static_cast<float>(x0);
  const float dy = fy - static_cast<float>(y0);

  // BORDER_REPLICATE clamp — matches default OpenCV resize edge behavior.
  x0 = clamp_i(x0, 0, src_w - 1);
  x1 = clamp_i(x1, 0, src_w - 1);
  y0 = clamp_i(y0, 0, src_h - 1);
  y1 = clamp_i(y1, 0, src_h - 1);

  const uint8_t * p00 = src + y0 * src_step + x0 * 3;
  const uint8_t * p01 = src + y0 * src_step + x1 * 3;
  const uint8_t * p10 = src + y1 * src_step + x0 * 3;
  const uint8_t * p11 = src + y1 * src_step + x1 * 3;

  // Bilinear weights
  const float w00 = (1.0f - dx) * (1.0f - dy);
  const float w01 = dx * (1.0f - dy);
  const float w10 = (1.0f - dx) * dy;
  const float w11 = dx * dy;

  const float inv255 = 1.0f / 255.0f;
  const int plane = dst_h * dst_w;
  const int idx = y * dst_w + x;

  // Destination channel order is R, G, B (TensorRT input).
  // If source is BGR, channel C_dst=0 (R) reads source byte index 2.
  // If source is RGB, channel C_dst=0 (R) reads source byte index 0.
  #pragma unroll
  for (int c_dst = 0; c_dst < 3; ++c_dst) {
    const int c_src = SrcIsBgr ? (2 - c_dst) : c_dst;
    const float v =
      (w00 * static_cast<float>(p00[c_src]) +
       w01 * static_cast<float>(p01[c_src]) +
       w10 * static_cast<float>(p10[c_src]) +
       w11 * static_cast<float>(p11[c_src])) * inv255;
    dst[c_dst * plane + idx] = v;
  }
}

}  // namespace

cudaError_t launch_preprocess(
  const uint8_t * src,
  int src_h, int src_w, int src_step,
  bool src_is_bgr,
  float * dst,
  int dst_h, int dst_w,
  cudaStream_t stream)
{
  // OpenCV scale convention.
  const float scale_x = static_cast<float>(src_w) / static_cast<float>(dst_w);
  const float scale_y = static_cast<float>(src_h) / static_cast<float>(dst_h);

  dim3 block(32, 8);
  dim3 grid((dst_w + block.x - 1) / block.x,
            (dst_h + block.y - 1) / block.y);

  if (src_is_bgr) {
    preprocess_kernel<true><<<grid, block, 0, stream>>>(
      src, src_h, src_w, src_step, dst, dst_h, dst_w, scale_x, scale_y);
  } else {
    preprocess_kernel<false><<<grid, block, 0, stream>>>(
      src, src_h, src_w, src_step, dst, dst_h, dst_w, scale_x, scale_y);
  }

  return cudaPeekAtLastError();
}

}  // namespace object_detection_cpp
