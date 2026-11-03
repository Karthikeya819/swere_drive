#include <swere_drive/drive_command_utils.hpp>
#include <cstdlib>
#include <iostream>

void check(bool condition) {
  if (!condition) { std::cerr << "Regression check failed\n"; std::exit(1); }
}
int main() {
  bool value = false;
  for (auto text : {"true", "True", "TRUE", "1"}) {
    check(swere_drive::ParseBoolean(text, value) && value);
  }
  for (auto text : {"false", "False", "FALSE", "0"}) {
    check(swere_drive::ParseBoolean(text, value) && !value);
  }
  check(!swere_drive::ParseBoolean("yes", value));
  check(!swere_drive::ParseBoolean("", value));
  swere_drive::CrabTransition transition;
  check(!transition.Ready(100));
  transition.Begin(10, 2);
  check(!transition.Ready(11.49));
  transition.Begin(11.49, -3);
  check(transition.Ready(11.5));
  check(transition.Speed() == -3);
  transition.Cancel();
  check(!transition.Ready(100));
  transition.Begin(20, 4);
  check(!transition.Ready(21));
  check(transition.Ready(21.5));
  check(!transition.Ready(0));
  check(!transition.Ready(100));
  // Simulated physics ticks complete the delay without any odometry polling.
  swere_drive::CrabTransition physics_transition;
  physics_transition.Begin(0, 2);
  check(!physics_transition.Ready(1.499));
  check(physics_transition.Ready(1.500));
  // A coarse physics step can cross the deadline but must not defer further.
  physics_transition.Cancel();
  physics_transition.Begin(2, 2);
  check(!physics_transition.Ready(3.49));
  check(physics_transition.Ready(3.51));
  std::cout << "All drive command regression checks passed\n";
}
