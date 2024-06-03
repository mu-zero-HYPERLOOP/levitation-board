#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "pwm_brake.h"
#include "sdc_brake.h"

levitation_state fsm::states::idle(levitation_command cmd, Duration time_since_last_transition) {

  if (levitation_command_ARM45 == cmd){
    return levitation_state_ARMING45;
  }

  // releases the brakes. 
  // This implies that if any brake is pulled the system has to return to the idle state
  // before going back to normal operation.
  sdc_brake::release_brake();
  pwm_brake::release_brake();

  sdc_brake::open();
  pwm_brake::stop();
  precharge_mosfet::open();
  feedthrough_mosfet::open();

  return levitation_state_IDLE;
}
