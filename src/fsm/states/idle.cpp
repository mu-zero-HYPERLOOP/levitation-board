#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

levitation_state fsm::states::idle(levitation_command cmd, Duration time_since_last_transition) {

  if (levitation_command_ARM45 == cmd){
    return levitation_state_ARMING45;
  }

  if (time_since_last_transition > 3_s){
    canzero_set_error_arming_failed(error_flag_OK);
    canzero_set_error_precharge_failed(error_flag_OK);
  }



  sdc_brake::release_brake();
  sdc_brake::open();

  pwm::disable_output();
  pwm::disable_trig1();

  precharge_mosfet::open();
  feedthrough_mosfet::open();

  return levitation_state_IDLE;
}
