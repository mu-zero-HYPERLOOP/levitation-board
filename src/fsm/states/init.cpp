#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"


levitation_state fsm::states::init(levitation_command cmd, Duration time_since_last_transition) {

  canzero_set_command(levitation_command_NONE);

  sdc_brake::open();

  pwm::disable_output();
  pwm::disable_trig1();


  precharge_mosfet::open();
  feedthrough_mosfet::open();

  return levitation_state_IDLE;
}
