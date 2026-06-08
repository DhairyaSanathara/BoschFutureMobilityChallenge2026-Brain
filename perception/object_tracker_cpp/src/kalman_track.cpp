#include "object_tracker_cpp/kalman_track.hpp"

#include <algorithm>

namespace object_tracker_cpp
{

namespace
{
// Tuned for BFMC scale (objects move at < 2 m/s, detections at ~30 Hz).
constexpr double POS_VAR   = 0.05;   // m^2
constexpr double VEL_VAR   = 0.5;    // (m/s)^2 — random walk on velocity
constexpr double MEAS_VAR  = 0.10;   // m^2 (≈ 30 cm depth-projection std)
constexpr double INIT_POS_VAR = 0.5; // m^2
constexpr double INIT_VEL_VAR = 1.0; // (m/s)^2
}  // namespace

KalmanTrack::KalmanTrack(std::uint16_t track_id,
                         std::uint8_t  class_id,
                         std::string   class_name,
                         double meas_x, double meas_y,
                         double initial_confidence)
: track_id_(track_id),
  class_id_(class_id),
  class_name_(std::move(class_name)),
  confidence_(initial_confidence)
{
  x_ << meas_x, meas_y, 0.0, 0.0;

  // Birth covariance: known position (to within sensor noise), unknown velocity.
  P_.setZero();
  P_(0,0) = INIT_POS_VAR;
  P_(1,1) = INIT_POS_VAR;
  P_(2,2) = INIT_VEL_VAR;
  P_(3,3) = INIT_VEL_VAR;

  F_.setIdentity();    // rebuilt with dt in predict()

  Q_.setZero();
  Q_(0,0) = POS_VAR;
  Q_(1,1) = POS_VAR;
  Q_(2,2) = VEL_VAR;
  Q_(3,3) = VEL_VAR;

  H_.setZero();
  H_(0,0) = 1.0;
  H_(1,1) = 1.0;

  R_.setIdentity();
  R_ *= MEAS_VAR;

  recompute_innovation_cov_();
}

void KalmanTrack::predict(double dt)
{
  dt = std::clamp(dt, 0.0, MAX_DT_);
  ++age_;
  ++time_since_update_;

  F_(0,2) = dt;
  F_(1,3) = dt;

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_ * dt;     // scale Q by dt for time-consistency
  P_ = 0.5 * (P_ + P_.transpose());
  recompute_innovation_cov_();

  // Once a confirmed track stops getting updates it slips into Lost state at
  // the tracker level (configurable max_age). We don't transition here — the
  // tracker owns lifecycle thresholds — but the bookkeeping is up to date.
}

void KalmanTrack::update(double meas_x, double meas_y, double confidence)
{
  Eigen::Vector2d z; z << meas_x, meas_y;
  const Eigen::Vector2d y = z - H_ * x_;                  // innovation
  const Eigen::Matrix<double,4,2> K = P_ * H_.transpose() * S_.inverse();
  x_ += K * y;
  const Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  P_ = (I - K * H_) * P_;
  P_ = 0.5 * (P_ + P_.transpose());
  recompute_innovation_cov_();

  ++hits_;
  time_since_update_ = 0;
  confidence_ = confidence;       // expose latest detection confidence to consumers
}

void KalmanTrack::mark_missed()
{
  // Predict() already incremented time_since_update_. The tracker decides
  // whether to flip our state to Lost / remove us.
  if (state_ == State::Confirmed && time_since_update_ > 0) {
    state_ = State::Lost;
  }
}

bool KalmanTrack::maybe_promote(std::uint64_t n_init)
{
  if (state_ == State::Tentative && hits_ >= n_init && time_since_update_ == 0) {
    state_ = State::Confirmed;
    return true;
  }
  // A lost track that suddenly re-matches comes back as Confirmed (its id is
  // preserved). This is the whole reason ByteTrack uses a two-stage match.
  if (state_ == State::Lost && time_since_update_ == 0) {
    state_ = State::Confirmed;
    return true;
  }
  return false;
}

void KalmanTrack::recompute_innovation_cov_()
{
  S_ = H_ * P_ * H_.transpose() + R_;
}

}  // namespace object_tracker_cpp
