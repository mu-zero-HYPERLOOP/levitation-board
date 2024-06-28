#include "airgap_transition.h"
#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

constexpr Duration STATE_TIMEOUT = 5_s;

// Invaraiant: None
levitation_state fsm::states::arming45(levitation_command cmd,
                                       Duration time_since_last_transition) {

  if (levitation_command_DISARM45 == cmd || levitation_command_ABORT == cmd) {
    return levitation_state_DISARMING45;
  }

  if (time_since_last_transition > STATE_TIMEOUT) {
    // timeout (in reality arm45 and precharge commands only have the delay
    // of communication between them so most likely less than 100ms in all cases.)
    // High delay here for manual control!
    canzero_set_command(levitation_command_DISARM45);
    canzero_set_error_arming_failed(error_flag_ERROR);
    return levitation_state_DISARMING45;
  }

  airgap_transition::transition_to_ground(0_s);

  if (levitation_command_PRECHARGE == cmd) {
    // precharge should only be send if all SDC switches are closed.
    return levitation_state_PRECHARGE;
  }

  pwm::control(PwmControl());
  pwm::enable_output();
  pwm::disable_trig1();

  if (!sdc_brake::request_close()) {
    // Failed to open SDC! (the brake was pulled)
    canzero_set_command(levitation_command_DISARM45);
    return levitation_state_DISARMING45;
  }
  precharge_mosfet::open();
  feedthrough_mosfet::open();

  return levitation_state_ARMING45;
}
