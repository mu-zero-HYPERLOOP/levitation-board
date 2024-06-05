#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

constexpr Duration MIN_STATE_TIME = 3_s;
constexpr Duration MAX_STATE_TIME = 5_s;
constexpr Voltage MIN_REQ_VOLTAGE = 40_V;

levitation_state fsm::states::precharge(levitation_command cmd, Duration time_since_last_transition) {
  
  if (levitation_command_DISARM45 == cmd || levitation_command_ABORT == cmd){
    return levitation_state_DISARMING45;
  }

  if (time_since_last_transition > MAX_STATE_TIME){
    canzero_set_command(levitation_command_DISARM45);
    canzero_set_error_precharge_failed(error_flag_ERROR);
    return levitation_state_DISARMING45;
  }

  if (time_since_last_transition > MIN_STATE_TIME && 
      canzero_get_vdc_voltage() >= static_cast<float>(MIN_REQ_VOLTAGE)){
    return levitation_state_READY;
  }

  pwm::control(PwmControl());
  pwm::enable_output();
  pwm::disable_trig0();
  pwm::disable_trig1();

  if (!sdc_brake::request_close()){
    // Failed to open SDC. (brake pulled).
    canzero_set_command(levitation_command_DISARM45);
    return levitation_state_DISARMING45;
  }

  precharge_mosfet::close();
  feedthrough_mosfet::close();

  return levitation_state_PRECHARGE;
}

