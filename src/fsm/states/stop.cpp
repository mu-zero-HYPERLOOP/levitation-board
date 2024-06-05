#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

levitation_state fsm::states::stop(levitation_command cmd,
                                   Duration time_since_last_transition) {

  if (levitation_command_DISARM45 == cmd){
    return levitation_state_DISARMING45;
  }

  // TODO : Determine if levitation has stopped completely
  
  bool control_stopped = false;
  if (control_stopped){
    return levitation_state_READY;
  }
  

  pwm::enable_output();
  // control set by isr.
  pwm::enable_trig0();
  pwm::enable_trig1();

  if (!sdc_brake::request_close()){
    // Failed to close SDC (brake pulled).
    canzero_set_command(levitation_command_DISARM45);
    return levitation_state_DISARMING45;
  }

  precharge_mosfet::open();
  feedthrough_mosfet::close();

  return levitation_state_STOP;
}
