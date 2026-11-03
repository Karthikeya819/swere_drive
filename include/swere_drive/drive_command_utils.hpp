#ifndef SWERE_DRIVE_COMMAND_UTILS_HPP
#define SWERE_DRIVE_COMMAND_UTILS_HPP

#include <algorithm>
#include <cctype>
#include <string>

namespace swere_drive {
// Match the boolean spellings used by SDF and the example URDF files.
inline bool ParseBoolean(std::string value, bool &result) {
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "true" || value == "1") { result = true; return true; }
  if (value == "false" || value == "0") { result = false; return true; }
  return false;
}

// A new crab command updates speed without restarting the steering delay.
// All calls are made by the plugin's single-threaded ROS executor.
class CrabTransition {
 public:
  void Begin(double now, float speed) {
    if (now < last_time_) Cancel();
    if (!active_) { ready_at_ = now + 1.5; active_ = true; }
    last_time_ = now;
    speed_ = speed;
  }
  void Cancel() { active_ = false; }
  bool Ready(double now) {
    // A world reset must not replay an old command when time catches up.
    if (now < last_time_) Cancel();
    last_time_ = now;
    return active_ && now >= ready_at_;
  }
  float Speed() const { return speed_; }
 private:
  bool active_ = false;
  double last_time_ = 0.0;
  double ready_at_ = 0.0;
  float speed_ = 0.0;
};
}  // namespace swere_drive
#endif
