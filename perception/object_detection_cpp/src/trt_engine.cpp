#include "object_detection_cpp/trt_engine.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace object_detection_cpp
{

void TrtEngine::Logger::log(Severity severity, const char * msg) noexcept
{
  if (severity == Severity::kINTERNAL_ERROR ||
      severity == Severity::kERROR ||
      severity == Severity::kWARNING)
  {
    std::cerr << "[TRT] " << msg << std::endl;
  }
}

namespace
{
void cuda_throw(cudaError_t err, const char * where)
{
  if (err != cudaSuccess) {
    std::ostringstream s;
    s << "CUDA error at " << where << ": " << cudaGetErrorString(err);
    throw std::runtime_error(s.str());
  }
}
}  // namespace

TrtEngine::TrtEngine(const std::string & engine_path, int input_h, int input_w)
: input_h_(input_h), input_w_(input_w),
  logger_(std::make_unique<Logger>())
{
  load_engine_(engine_path);
  configure_io_();
}

TrtEngine::~TrtEngine()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
  if (d_input_)  { cudaFree(d_input_);   d_input_ = nullptr; }
  if (d_output_) { cudaFree(d_output_);  d_output_ = nullptr; }
  if (h_output_) { cudaFreeHost(h_output_); h_output_ = nullptr; }
}

void TrtEngine::load_engine_(const std::string & path)
{
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) {
    throw std::runtime_error("cannot open TRT engine: " + path);
  }
  const std::streamsize size = f.tellg();
  f.seekg(0, std::ios::beg);
  std::vector<char> buf(size);
  if (!f.read(buf.data(), size)) {
    throw std::runtime_error("failed to read TRT engine file");
  }

  runtime_.reset(nvinfer1::createInferRuntime(*logger_));
  if (!runtime_) {
    throw std::runtime_error("nvinfer1::createInferRuntime failed");
  }

  engine_.reset(runtime_->deserializeCudaEngine(buf.data(), size));
  if (!engine_) {
    throw std::runtime_error("deserializeCudaEngine failed");
  }

  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("createExecutionContext failed");
  }
}

void TrtEngine::configure_io_()
{
  // Discover input/output tensor names by IO mode (TRT 10 API).
  const int nb = engine_->getNbIOTensors();
  for (int i = 0; i < nb; ++i) {
    const char * name = engine_->getIOTensorName(i);
    const auto mode = engine_->getTensorIOMode(name);
    if (mode == nvinfer1::TensorIOMode::kINPUT && input_name_.empty()) {
      input_name_ = name;
    } else if (mode == nvinfer1::TensorIOMode::kOUTPUT && output_name_.empty()) {
      output_name_ = name;
    }
  }
  if (input_name_.empty() || output_name_.empty()) {
    throw std::runtime_error("engine missing single input/output tensor");
  }

  // Set static input shape (NCHW) if engine has dynamic dim(s).
  const nvinfer1::Dims4 in_dims{1, 3, input_h_, input_w_};
  if (!context_->setInputShape(input_name_.c_str(), in_dims)) {
    throw std::runtime_error("setInputShape failed");
  }

  // Probe output shape
  auto out_shape = context_->getTensorShape(output_name_.c_str());

  // YOLOv8 output is (1, 4 + nc, N). Tolerate (1, N, 4 + nc) by inspecting dims.
  if (out_shape.nbDims != 3 || out_shape.d[0] != 1) {
    std::ostringstream s;
    s << "unexpected output shape (nbDims=" << out_shape.nbDims << ")";
    throw std::runtime_error(s.str());
  }
  output_channels_    = out_shape.d[1];
  output_predictions_ = out_shape.d[2];

  // Bytes
  input_bytes_  = static_cast<size_t>(in_dims.d[0]) * in_dims.d[1]
                  * in_dims.d[2] * in_dims.d[3] * sizeof(float);
  output_bytes_ = static_cast<size_t>(out_shape.d[0]) * out_shape.d[1]
                  * out_shape.d[2] * sizeof(float);

  // Allocate device + pinned host buffers.
  cuda_throw(cudaMalloc(reinterpret_cast<void**>(&d_input_),  input_bytes_),  "cudaMalloc(d_input)");
  cuda_throw(cudaMalloc(reinterpret_cast<void**>(&d_output_), output_bytes_), "cudaMalloc(d_output)");
  cuda_throw(cudaHostAlloc(reinterpret_cast<void**>(&h_output_), output_bytes_, cudaHostAllocDefault),
             "cudaHostAlloc(h_output)");

  // Stream with non-blocking flag so it can overlap with the implicit stream.
  cuda_throw(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
             "cudaStreamCreateWithFlags");

  // Bind tensor addresses to the context.
  if (!context_->setTensorAddress(input_name_.c_str(),  d_input_)) {
    throw std::runtime_error("setTensorAddress(input) failed");
  }
  if (!context_->setTensorAddress(output_name_.c_str(), d_output_)) {
    throw std::runtime_error("setTensorAddress(output) failed");
  }
}

void TrtEngine::enqueue()
{
  // The preprocess kernel must have been launched on this same stream already.
  if (!context_->enqueueV3(stream_)) {
    throw std::runtime_error("enqueueV3 failed");
  }
  // Async D2H of the output into pinned host memory.
  cuda_throw(cudaMemcpyAsync(h_output_, d_output_, output_bytes_,
                             cudaMemcpyDeviceToHost, stream_),
             "cudaMemcpyAsync(DtoH output)");
}

void TrtEngine::sync()
{
  cuda_throw(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
}

void TrtEngine::warmup(int runs)
{
  // Zero the input on device, then enqueue+sync a few times.
  cuda_throw(cudaMemsetAsync(d_input_, 0, input_bytes_, stream_), "warmup memset");
  for (int i = 0; i < runs; ++i) {
    enqueue();
    sync();
  }
}

}  // namespace object_detection_cpp
