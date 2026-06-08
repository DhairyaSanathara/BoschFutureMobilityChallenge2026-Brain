#ifndef OBJECT_DETECTION_CPP__TRT_ENGINE_HPP_
#define OBJECT_DETECTION_CPP__TRT_ENGINE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <cuda_runtime.h>
#include <NvInfer.h>

namespace object_detection_cpp
{

// Lightweight RAII wrapper around a TensorRT 10 engine and execution context
// configured for a single-input / single-output YOLOv8 model.
//
// Owns:
//   - device input buffer  (float32, 1*3*H*W)
//   - device output buffer (float32, 1*(4+nc)*N)
//   - pinned host output buffer (for DtoH async)
//   - one CUDA stream
//
// The preprocess kernel writes directly into device_input(), then call
// enqueue() to run inference and copy the result into host_output().
// host_output() is only valid after sync() returns.
class TrtEngine
{
public:
  TrtEngine(const std::string & engine_path,
            int input_h, int input_w);
  ~TrtEngine();

  TrtEngine(const TrtEngine &) = delete;
  TrtEngine & operator=(const TrtEngine &) = delete;

  void enqueue();   // H -> kernel write expected, run inference + DtoH
  void sync();      // block until DtoH completes
  void warmup(int runs);

  float *      device_input()   const { return d_input_; }
  const float * host_output()    const { return h_output_; }
  cudaStream_t stream()          const { return stream_; }

  int input_h() const { return input_h_; }
  int input_w() const { return input_w_; }

  // Output dims as queried from the engine: (1, channels, num_predictions)
  int output_channels()    const { return output_channels_; }
  int output_predictions() const { return output_predictions_; }

private:
  class Logger : public nvinfer1::ILogger
  {
  public:
    void log(Severity severity, const char * msg) noexcept override;
  };

  void load_engine_(const std::string & path);
  void configure_io_();

  int input_h_;
  int input_w_;

  std::unique_ptr<Logger> logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  std::string input_name_;
  std::string output_name_;

  size_t input_bytes_{0};
  size_t output_bytes_{0};
  int output_channels_{0};
  int output_predictions_{0};

  float * d_input_{nullptr};
  float * d_output_{nullptr};
  float * h_output_{nullptr};      // pinned

  cudaStream_t stream_{nullptr};
};

}  // namespace object_detection_cpp

#endif  // OBJECT_DETECTION_CPP__TRT_ENGINE_HPP_
