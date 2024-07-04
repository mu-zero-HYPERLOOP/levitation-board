#include "airgap_transition.h"
#include "control.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

constexpr Duration MIN_STATE_TIME = 2_s;

levitation_state fsm::states::disarming45(levitation_command cmd, Duration time_since_last_transition) {

  if (time_since_last_transition > MIN_STATE_TIME){
    return levitation_state_IDLE;
  }

  airgap_transition::transition_to_ground(0_s);

  pwm::control(GuidancePwmControl());
  pwm::enable_output();
  pwm::disable_trig1();


  sdc_brake::open();

  precharge_mosfet::open();
  feedthrough_mosfet::open(); //maybe this should remain open not sure!

  return levitation_state_DISARMING45;
}
