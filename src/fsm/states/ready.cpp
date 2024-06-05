#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

levitation_state fsm::states::ready(levitation_command cmd, Duration time_since_last_transition) {

  if (levitation_command_DISARM45 == cmd || levitation_command_ABORT == cmd){
    return levitation_state_DISARMING45;
  }

  if (levitation_command_START == cmd){
    return levitation_state_START;
  }

  pwm::control(PwmControl());
  pwm::enable_output();
  pwm::disable_trig0();
  pwm::disable_trig1();

  if (!sdc_brake::request_close()){
    // Close SDC Failed. (brake pulled).
    canzero_set_command(levitation_command_DISARM45);
    return levitation_state_DISARMING45;
  }

  precharge_mosfet::close();
  feedthrough_mosfet::open();

  return levitation_state_READY;
}
