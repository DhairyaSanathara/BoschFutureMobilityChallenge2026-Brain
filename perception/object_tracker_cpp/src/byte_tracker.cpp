#include "object_tracker_cpp/byte_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace object_tracker_cpp
{

ByteTracker::ByteTracker(ByteTrackerParams params)
: params_(params)
{}

std::uint16_t ByteTracker::alloc_id_()
{
  // Prefer reused ids to keep the live id-space dense.
  if (!free_ids_.empty()) {
    const auto id = free_ids_.front();
    free_ids_.pop_front();
    return id;
  }
  if (next_id_ <= 65535u) {
    return static_cast<std::uint16_t>(next_id_++);
  }
  // Saturation: 65 535 distinct tracks ever existed with none yet recycled.
  // Wrap defensively to 1 (0 is reserved as the "unassigned" sentinel).
  // This branch is unreachable at BFMC scale: even at 30 new tracks/sec
  // sustained it would take ~36 minutes to exhaust the range without any
  // ever being recycled, and tracks live ~1 s so recycling dominates.
  return 1;
}

void ByteTracker::free_id_(std::uint16_t id)
{
  if (id != 0) free_ids_.push_back(id);
}

std::vector<const KalmanTrack *> ByteTracker::step(
    double dt, std::vector<Detection> detections)
{
  // Fresh lifecycle log for this step. Cleared at the top so the node sees
  // *only* what happened in this exact call.
  last_births_.clear();
  last_deaths_.clear();

  // ---- 1. Predict every existing track to the current frame ---------------
  for (auto & t : tracks_) {
    t->predict(dt);
  }

  // ---- 2. Split detections into high- and low-confidence pools ------------
  std::vector<std::size_t> high_dets, low_dets;
  high_dets.reserve(detections.size());
  low_dets .reserve(detections.size());
  for (std::size_t i = 0; i < detections.size(); ++i) {
    const float c = detections[i].confidence;
    if (c >= params_.conf_high)      high_dets.push_back(i);
    else if (c >= params_.conf_low)  low_dets .push_back(i);
    // < conf_low ⇒ silently dropped
  }

  std::vector<std::size_t> all_track_idxs(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) all_track_idxs[i] = i;

  std::vector<std::pair<std::size_t, std::size_t>> matches;
  matches.reserve(tracks_.size());

  // ---- 3. Stage 1: high-conf detections vs all tracks ---------------------
  match_pool_(detections, all_track_idxs, high_dets, matches);

  // ---- 4. Stage 2: low-conf detections vs still-unmatched tracks ----------
  match_pool_(detections, all_track_idxs, low_dets, matches);

  // ---- 5. Apply matches ---------------------------------------------------
  for (auto & [ti, di] : matches) {
    const auto & d = detections[di];
    tracks_[ti]->update(d.x, d.y, d.confidence);
    tracks_[ti]->maybe_promote(params_.n_init);
  }

  // ---- 6. Spawn new tracks from unmatched HIGH-conf detections only -------
  //         (low-conf detections cannot give birth to a track — they can
  //          only refresh an existing one in the stage-2 match above.)
  for (auto di : high_dets) {
    const auto & d = detections[di];
    auto id = alloc_id_();
    tracks_.push_back(std::make_unique<KalmanTrack>(
      id, d.class_id, d.class_name, d.x, d.y, d.confidence));
    ++total_created_;
    last_births_.push_back({id, d.class_id, d.x, d.y});
  }

  // ---- 7. Mark unmatched tracks and prune dead ones -----------------------
  for (auto ti : all_track_idxs) tracks_[ti]->mark_missed();

  // Remove tracks that have exceeded their lifetime budget. Confirmed tracks
  // get a longer (max_lost_frames) grace before deletion so a brief occlusion
  // can be recovered by a future low-conf match. Tentative tracks die fast.
  auto kill_pred = [this](const std::unique_ptr<KalmanTrack> & t) {
    if (t->state() == KalmanTrack::State::Tentative &&
        t->time_since_update() > params_.n_init) return true;
    if (t->state() == KalmanTrack::State::Lost &&
        t->time_since_update() > params_.max_lost_frames) return true;
    if (t->time_since_update() > params_.max_age) return true;
    return false;
  };
  for (auto & t : tracks_) {
    if (kill_pred(t)) {
      // Only Confirmed-or-Lost deaths feed the ID-switch heuristic; a dying
      // Tentative was never a real object so its disappearance is not an
      // identity event.
      if (t->state() != KalmanTrack::State::Tentative) {
        last_deaths_.push_back({t->track_id(), t->class_id(), t->x(), t->y()});
      }
      free_id_(t->track_id());
    }
  }
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(), kill_pred),
    tracks_.end());

  // ---- 8. Output: confirmed tracks only -----------------------------------
  std::vector<const KalmanTrack *> out;
  out.reserve(tracks_.size());
  for (const auto & t : tracks_) {
    if (t->state() == KalmanTrack::State::Confirmed) out.push_back(t.get());
  }
  return out;
}

void ByteTracker::match_pool_(
    const std::vector<Detection> & dets,
    std::vector<std::size_t> & track_idxs,
    std::vector<std::size_t> & det_idxs,
    std::vector<std::pair<std::size_t, std::size_t>> & matches) const
{
  // Greedy O(N·M) nearest-neighbour with a class lock and a hard distance
  // gate. At BFMC scale (N, M < 20) this beats Hungarian on both clarity
  // and speed; a wrong assignment that survives the gate would be a
  // pathological birds-eye-view geometry we don't see on the track.
  while (!track_idxs.empty() && !det_idxs.empty()) {
    double best_d = std::numeric_limits<double>::infinity();
    std::size_t best_ti = 0, best_di = 0;
    bool found = false;

    for (auto ti : track_idxs) {
      const auto & t = *tracks_[ti];
      for (auto di : det_idxs) {
        const auto & d = dets[di];
        if (d.class_id != t.class_id()) continue;         // class lock
        const double dx = d.x - t.x();
        const double dy = d.y - t.y();
        const double dist = std::sqrt(dx*dx + dy*dy);
        if (dist > params_.max_assoc_dist) continue;      // hard gate
        if (dist < best_d) { best_d = dist; best_ti = ti; best_di = di; found = true; }
      }
    }
    if (!found) break;

    matches.emplace_back(best_ti, best_di);

    track_idxs.erase(std::remove(track_idxs.begin(), track_idxs.end(), best_ti),
                     track_idxs.end());
    det_idxs  .erase(std::remove(det_idxs  .begin(), det_idxs  .end(), best_di),
                     det_idxs  .end());
  }
}

}  // namespace object_tracker_cpp
