#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

constexpr Duration MIN_STATE_TIME = 3_s;
constexpr Duration MAX_STATE_TIME = 5_s;

// Just to check that there actually is voltage on the vdc plane!
// Because of constant load on the 45V line (15V buck).
// The precharge voltage will never reach 45V!
// 15V has nothing to do with 15V buck!
constexpr Voltage MIN_REQ_VOLTAGE = 15_V;
constexpr bool REQUIRE_VOLTAGE = false;

levitation_state fsm::states::precharge(levitation_command cmd,
                                        Duration time_since_last_transition) {

  if (levitation_command_DISARM45 == cmd || levitation_command_ABORT == cmd) {
    return levitation_state_DISARMING45;
  }

  if (time_since_last_transition > MAX_STATE_TIME) {
    canzero_set_command(levitation_command_DISARM45);
    canzero_set_error_precharge_failed(error_flag_ERROR);
    return levitation_state_DISARMING45;
  }

  if (time_since_last_transition > MIN_STATE_TIME &&
      (canzero_get_vdc_voltage() >= static_cast<float>(MIN_REQ_VOLTAGE) ||
       !REQUIRE_VOLTAGE)) {
    return levitation_state_READY;
  }

  pwm::control(PwmControl());
  pwm::enable_output();
  pwm::disable_trig1();

  if (!sdc_brake::request_close()) {
    // Failed to open SDC. (brake pulled).
    canzero_set_command(levitation_command_DISARM45);
    return levitation_state_DISARMING45;
  }

  precharge_mosfet::close();
  feedthrough_mosfet::open();

  return levitation_state_PRECHARGE;
}
