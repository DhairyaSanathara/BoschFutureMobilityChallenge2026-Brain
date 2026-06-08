#ifndef OBJECT_TRACKER_CPP__KALMAN_TRACK_HPP_
#define OBJECT_TRACKER_CPP__KALMAN_TRACK_HPP_

#include <cstdint>
#include <string>

#include <Eigen/Dense>

namespace object_tracker_cpp
{

// One KF-backed track for a single physical object. State is a 4D
// constant-velocity model in the same frame as the input detections:
//   x_state = [x, y, vx, vy]^T
// Measurement is z = [x, y]^T (the 3D detector projects to vehicle frame and
// we only track planar motion — the z component of position has no useful
// velocity at this scale).
//
// The track owns its lifecycle bookkeeping (hits, time_since_update, age,
// state enum) so the tracker can stay stateless across frames except for the
// list of these objects.
class KalmanTrack
{
public:
  enum class State : std::uint8_t { Tentative = 0, Confirmed = 1, Lost = 2 };

  KalmanTrack(std::uint16_t track_id,
              std::uint8_t  class_id,
              std::string   class_name,
              double meas_x, double meas_y,
              double initial_confidence);

  // Constant-velocity prediction. dt clamped internally to [0, MAX_DT_].
  void predict(double dt);

  // Measurement update. Resets time_since_update, increments hits, updates
  // last-seen confidence.
  void update(double meas_x, double meas_y, double confidence);

  // Called once per frame on every track that was NOT matched. Increments
  // age + time_since_update; may transition Confirmed→Lost.
  void mark_missed();

  // Promote a tentative track to confirmed once it has accumulated enough
  // consecutive hits to be statistically distinguishable from noise.
  bool maybe_promote(std::uint64_t n_init);

  // Accessors used by the tracker for matching / output.
  double x()  const { return x_(0); }
  double y()  const { return x_(1); }
  double vx() const { return x_(2); }
  double vy() const { return x_(3); }

  // 2x2 measurement-space covariance H P H^T + R, exposed for Mahalanobis-
  // style gating. (Stored cached so the per-tick computation is O(1).)
  const Eigen::Matrix2d & innovation_cov() const { return S_; }

  std::uint16_t track_id()    const { return track_id_; }
  std::uint8_t class_id()    const { return class_id_; }
  const std::string & class_name() const { return class_name_; }
  float        confidence()  const { return static_cast<float>(confidence_); }
  State        state()       const { return state_; }
  std::uint64_t hits()       const { return hits_; }
  std::uint64_t time_since_update() const { return time_since_update_; }

  // Single global knob for the per-step prediction dt clamp; protects against
  // huge dt gaps when the detector restarts.
  static constexpr double MAX_DT_ = 0.5;

private:
  // Refresh S_ = H P H^T + R after predict / update.
  void recompute_innovation_cov_();

  Eigen::Vector4d x_;        // state
  Eigen::Matrix4d P_;        // state covariance
  Eigen::Matrix4d F_;        // state transition (rebuilt each predict for current dt)
  Eigen::Matrix4d Q_;        // process noise (constant)
  Eigen::Matrix<double,2,4> H_;     // measurement matrix
  Eigen::Matrix2d R_;        // measurement noise (constant)
  Eigen::Matrix2d S_;        // cached innovation covariance

  std::uint16_t track_id_;
  std::uint8_t  class_id_;
  std::string   class_name_;
  double        confidence_;
  State         state_{State::Tentative};
  std::uint64_t hits_{1};                 // counts updates including birth
  std::uint64_t age_{1};                  // counts predict() invocations + 1
  std::uint64_t time_since_update_{0};
};

}  // namespace object_tracker_cpp

#endif  // OBJECT_TRACKER_CPP__KALMAN_TRACK_HPP_
