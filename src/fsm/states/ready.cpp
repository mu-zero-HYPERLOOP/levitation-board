#include "airgap_transition.h"
#include "canzero/canzero.h"
#include "control.h"
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

  control::reset();

  airgap_transition::transition_to_ground(0_s);


  const float dutyLL = 0.5;
  const float dutyLR = 0.5;
  const float dutyRL = 0.5;
  const float dutyRR = 0.5;

  GuidancePwmControl pwmControl{};
  pwmControl.left_l = dutyLL;
  pwmControl.left_r = dutyLR;
  pwmControl.right_l = dutyRL;
  pwmControl.right_r = dutyRR;
  pwm::control(pwmControl);
  pwm::enable_output();
  pwm::disable_trig1();

  if (!sdc_brake::request_close()){
    // Close SDC Failed. (brake pulled).
    canzero_set_command(levitation_command_DISARM45);
    return levitation_state_DISARMING45;
  }

  precharge_mosfet::open();
  feedthrough_mosfet::close();

  return levitation_state_READY;
}
