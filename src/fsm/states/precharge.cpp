#include "airgap_transition.h"
#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "print.h"
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
    debugPrintf("ERROR_HANDLING: DISARM");
    return levitation_state_DISARMING45;
  }

  airgap_transition::transition_to_ground(0_s);

  if (time_since_last_transition > MAX_STATE_TIME) {
    canzero_set_command(levitation_command_DISARM45);
    canzero_set_error_precharge_failed(error_flag_ERROR);
    debugPrintf("ERROR_HANDLING: TIMEOUT");
    return levitation_state_DISARMING45;
  }

  if (time_since_last_transition > MIN_STATE_TIME){
    return levitation_state_READY;
  }

  pwm::control(PwmControl());
  pwm::enable_output();
  pwm::disable_trig1();

  if (!sdc_brake::request_close()) {
    // Failed to open SDC. (brake pulled).
    canzero_set_command(levitation_command_DISARM45);
    debugPrintf("ERROR_HANDLING: SDC_REQUEST_FAILED");
    return levitation_state_DISARMING45;
  }

  precharge_mosfet::close();
  feedthrough_mosfet::open();

  return levitation_state_PRECHARGE;
}
