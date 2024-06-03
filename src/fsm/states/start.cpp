#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "pwm_brake.h"
#include "sdc_brake.h"

levitation_state fsm::states::start(levitation_command cmd,
                                    Duration time_since_last_transition) {

  if (levitation_command_DISARM45 == cmd){
    return levitation_state_DISARMING45;
  }

  if (levitation_command_STOP == cmd || levitation_command_ABORT == cmd){
    return levitation_state_STOP;
  }

  // TODO : Set target airgap over a sigmoid. Realtive to time_since_last_transition!

  bool levitation_stable = false;
  if (levitation_stable){
    return levitation_state_CONTROL;
  }

  if (!sdc_brake::request_close()){
    // Failed to close SDC (brake pulled).
    canzero_set_command(levitation_command_DISARM45);
    return levitation_state_DISARMING45;
  }
  if (!pwm_brake::request_start()){
    // Failed to start PWM (brake pulled).
    canzero_set_command(levitation_command_STOP);
    return levitation_state_READY;
  }

  precharge_mosfet::open();
  feedthrough_mosfet::close();

  return levitation_state_START;
}
