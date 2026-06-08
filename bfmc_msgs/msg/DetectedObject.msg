uint8 id
string class_name
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 confidence

# Phase 3.1: stable per-object tracker identity. 0 ⇒ untracked (detector raw
# output); nonzero ⇒ assigned by object_tracker_cpp and stable across frames
# for the lifetime of the corresponding KalmanTrack. Appended at the end so
# legacy consumers that only read the original 5 fields continue to work
# unchanged. uint16 sized for race-day headroom (~65k distinct ids).
uint16 track_id
