#include "airgap_transition.h"
#include "canzero/canzero.h"
#include "control.h"
#include "feedthrough_mosfet.h"
#include "print.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

levitation_state fsm::states::ready(levitation_command cmd, Duration time_since_last_transition) {

  if (levitation_command_DISARM45 == cmd || levitation_command_ABORT == cmd){
    debugPrintf("ERROR_HANDLING: DISARM | ABORT");
    return levitation_state_DISARMING45;
  }

  if (levitation_command_START == cmd){
    return levitation_state_START;
  }

  control::reset();

  airgap_transition::transition_to_ground(0_s);


  pwm::control(GuidancePwmControl{});
  pwm::enable_trig1();

  if (!sdc_brake::request_close()){
    // Close SDC Failed. (brake pulled).
    debugPrintf("ERROR_HANDLING: SDC FAILED TO CLOSE");
    canzero_set_command(levitation_command_DISARM45);
    return levitation_state_DISARMING45;
  }

  precharge_mosfet::open();
  feedthrough_mosfet::close();

  return levitation_state_READY;
}
