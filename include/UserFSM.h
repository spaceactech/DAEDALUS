#ifndef ROCKET_AVIONICS_TEMPLATE_USERFSM_H
#define ROCKET_AVIONICS_TEMPLATE_USERFSM_H

#include "FreeRTOS.h"


#include <cstdint>
#include <cstdlib>

enum class UserState : uint8_t {
  STARTUP = 0,
  IDLE_SAFE,
  LAUNCH_PAD,
  ASCENT,
  APOGEE,
  DESCENT,
  PROBE_REALEASE,
  PAYLOAD_REALEASE,
  LANDED,

  POWERED,
  COASTING,
  DROGUE_DEPLOY,
  DROGUE_DESCEND,
  MAIN_DEPLOY,
  MAIN_DESCEND,
  RECOVERED_SAFE
};

inline const char *state_string(const UserState state) {
  switch (state) {
    case UserState::STARTUP:
      return "STARTUP";
    case UserState::IDLE_SAFE:
      return "IDLE_SAFE";
    case UserState::LAUNCH_PAD:
      return "LAUNCH_PAD";
    case UserState::ASCENT:
      return "ASCENT";
    case UserState::APOGEE:
      return "APOGEE";
    case UserState::DESCENT:
      return "DESCENT";
    case UserState::PROBE_REALEASE:
      return "PROBE_REALEASE";
    case UserState::PAYLOAD_REALEASE:
      return "PAYLOAD_REALEASE";
    case UserState::LANDED:
      return "LANDED";
      
    case UserState::POWERED:
      return "POWERED";
    case UserState::COASTING:
      return "COASTING";
    case UserState::DROGUE_DEPLOY:
      return "DROG_DEPL";
    case UserState::DROGUE_DESCEND:
      return "DROG_DESC";
    case UserState::MAIN_DEPLOY:
      return "MAIN_DEPL";
    case UserState::MAIN_DESCEND:
      return "MAIN_DESC";
    case UserState::RECOVERED_SAFE:
      return "REC_SAFE";
    default:
      __builtin_unreachable();
  }
}

class UserFSM {
  UserState state_{};
  UserState prev_state_{};

public:
  [[nodiscard]] UserState state() const {
    return state_;
  }

  void transfer(const UserState new_state) {
    prev_state_ = state_;
    state_      = new_state;
  }

  /**
   * For checking if the state has been changed.
   *
   * @return
   */
  bool on_enter() {
    if (state_ == prev_state_)
      return false;

    prev_state_ = state_;
    return true;
  }
};

extern void EvalFSM();

#endif  //ROCKET_AVIONICS_TEMPLATE_USERFSM_H
