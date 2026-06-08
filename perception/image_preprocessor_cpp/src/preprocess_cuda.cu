// =============================================================================
// preprocess_cuda.cu -- fused resize + sRGB->Lab L kernel for BFMC preprocess
// =============================================================================
// Why this exists:
//   The Python image_preprocessor was bottlenecked on a 1280x720 -> 640x360
//   CPU resize (~10 ms) followed by a CPU cvtColor BGR2Lab (~4 ms) for each
//   frame. On a Jetson Orin Nano the iGPU was sitting at 0% while the CPU
//   was eating 60% running these two passes; this kernel offloads both into
//   one fused launch and frees the CPU to run CLAHE + Sobel + morph at
//   real-time rates.
// =============================================================================

#include "image_preprocessor_cpp/preprocess_cuda.hpp"

#include <cmath>
#include <cstdint>
#include <atomic>

namespace image_preprocessor_cpp
{

// --- sRGB inverse-companding LUT ------------------------------------------
//
// One float per uint8 input (4 KiB total). Matches OpenCV's sRGB inverse
// gamma exactly: linearise(c/255) where
//   linearise(t) = t/12.92             if t <= 0.04045
//                = ((t+0.055)/1.055)^2.4 otherwise
__constant__ float c_srgb_lut[256];

static std::atomic<bool> g_constants_initialised{false};

cudaError_t init_srgb_constants()
{
  // Idempotent; only the first call actually copies to __constant__ memory.
  if (g_constants_initialised.load(std::memory_order_acquire)) {
    return cudaSuccess;
  }
  float host_lut[256];
  for (int i = 0; i < 256; ++i) {
    const float t = static_cast<float>(i) / 255.f;
    host_lut[i] = (t <= 0.04045f)
                    ? (t / 12.92f)
                    : powf((t + 0.055f) / 1.055f, 2.4f);
  }
  const auto err = cudaMemcpyToSymbol(c_srgb_lut, host_lut, sizeof(host_lut));
  if (err == cudaSuccess) {
    g_constants_initialised.store(true, std::memory_order_release);
  }
  return err;
}

// --- Lab f() branch helper -------------------------------------------------
__device__ __forceinline__ float lab_branch(float t)
{
  // Standard CIE Lab piecewise. cbrtf on Orin is hardware-native via __frsqrt-style fast path.
  return (t > 0.008856f) ? cbrtf(t) : fmaf(7.787f, t, 16.f / 116.f);
}

// --- main fused kernel -----------------------------------------------------
__global__ void resize_bgr_to_lab_l_kernel(
  const uint8_t * __restrict__ src,
  int src_h, int src_w, int src_step,
  int src_is_bgr,
  uint8_t * __restrict__ dst_l,
  int dst_h, int dst_w)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= dst_w || y >= dst_h) {
    return;
  }

  // OpenCV pixel-center convention for INTER_LINEAR.
  const float scale_x = static_cast<float>(src_w) / static_cast<float>(dst_w);
  const float scale_y = static_cast<float>(src_h) / static_cast<float>(dst_h);
  const float src_x = (static_cast<float>(x) + 0.5f) * scale_x - 0.5f;
  const float src_y = (static_cast<float>(y) + 0.5f) * scale_y - 0.5f;

  const int x0 = static_cast<int>(floorf(src_x));
  const int y0 = static_cast<int>(floorf(src_y));
  const float fx = src_x - static_cast<float>(x0);
  const float fy = src_y - static_cast<float>(y0);

  // BORDER_REPLICATE clamp.
  const int x0c = max(0, min(src_w - 1, x0));
  const int x1c = max(0, min(src_w - 1, x0 + 1));
  const int y0c = max(0, min(src_h - 1, y0));
  const int y1c = max(0, min(src_h - 1, y0 + 1));

  const uint8_t * p00 = src + y0c * src_step + x0c * 3;
  const uint8_t * p01 = src + y0c * src_step + x1c * 3;
  const uint8_t * p10 = src + y1c * src_step + x0c * 3;
  const uint8_t * p11 = src + y1c * src_step + x1c * 3;

  // Bilinear weights.
  const float w00 = (1.f - fx) * (1.f - fy);
  const float w01 = fx        * (1.f - fy);
  const float w10 = (1.f - fx) * fy;
  const float w11 = fx        * fy;

  // Per-channel interpolation (channel 0/1/2 ordering depends on src encoding).
  const float c0 = w00 * p00[0] + w01 * p01[0] + w10 * p10[0] + w11 * p11[0];
  const float c1 = w00 * p00[1] + w01 * p01[1] + w10 * p10[1] + w11 * p11[1];
  const float c2 = w00 * p00[2] + w01 * p01[2] + w10 * p10[2] + w11 * p11[2];

  // Round to uint8 to match OpenCV's resize->convert behaviour exactly.
  const int b_i = src_is_bgr ? static_cast<int>(c0 + 0.5f) : static_cast<int>(c2 + 0.5f);
  const int g_i =                    static_cast<int>(c1 + 0.5f);
  const int r_i = src_is_bgr ? static_cast<int>(c2 + 0.5f) : static_cast<int>(c0 + 0.5f);

  const int b_u = max(0, min(255, b_i));
  const int g_u = max(0, min(255, g_i));
  const int r_u = max(0, min(255, r_i));

  const float r_lin = c_srgb_lut[r_u];
  const float g_lin = c_srgb_lut[g_u];
  const float b_lin = c_srgb_lut[b_u];

  // CIE Y of D65-XYZ from linear sRGB.
  const float Y = fmaf(0.212671f, r_lin, fmaf(0.715160f, g_lin, 0.072169f * b_lin));

  // L* in [0..100]. Yn = 1.0 for D65.
  const float L_f = (Y > 0.008856f)
                      ? fmaf(116.f, lab_branch(Y), -16.f)
                      : 903.3f * Y;

  // OpenCV stores L as L * 255 / 100 (i.e. L * 2.55) in CV_8U Lab.
  const float L_8u_f = L_f * 2.55f;
  const int L_8u_i = static_cast<int>(L_8u_f + 0.5f);
  dst_l[y * dst_w + x] = static_cast<uint8_t>(max(0, min(255, L_8u_i)));
}

cudaError_t launch_resize_bgr_to_lab_l(
  const uint8_t * d_src,
  int src_h, int src_w, int src_step,
  bool src_is_bgr,
  uint8_t * d_dst_l,
  int dst_h, int dst_w,
  cudaStream_t stream)
{
  // 16x16 threads / block: 256 threads; good occupancy on SM 8.7 (Orin).
  dim3 block(16, 16, 1);
  dim3 grid(
    static_cast<unsigned>((dst_w + block.x - 1) / block.x),
    static_cast<unsigned>((dst_h + block.y - 1) / block.y),
    1);

  resize_bgr_to_lab_l_kernel<<<grid, block, 0, stream>>>(
    d_src, src_h, src_w, src_step,
    src_is_bgr ? 1 : 0,
    d_dst_l, dst_h, dst_w);

  return cudaPeekAtLastError();
}

}  // namespace image_preprocessor_cpp
