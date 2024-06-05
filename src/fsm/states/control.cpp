#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"


levitation_state fsm::states::control(levitation_command cmd, Duration time_since_last_transition) {

  if (levitation_command_DISARM45 == cmd){
    return levitation_state_DISARMING45;
  }

  if (levitation_command_STOP == cmd || levitation_command_ABORT == cmd){
    return levitation_state_READY;
  }
  pwm::enable_output();
  // pwm control set by isr.
  pwm::enable_trig0();
  pwm::enable_trig1();

  if (!sdc_brake::request_close()){
    // Failed to open SDC. (brake pulled).
    canzero_set_command(levitation_command_DISARM45);
    return levitation_state_DISARMING45;
  }
  
  precharge_mosfet::open();
  feedthrough_mosfet::close();

  return levitation_state_CONTROL;
}
