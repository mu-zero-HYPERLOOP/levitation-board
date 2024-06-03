#include "fsm/fsm.h"
#include "canzero/canzero.h"

#include "fsm/error_handling.h"
#include "fsm/states.h"
#include "util/timestamp.h"

static Timestamp fsm_last_transition = Timestamp::now();

void fsm::begin() {
  fsm_last_transition = Timestamp::now();
  canzero_set_state(levitation_state_INIT);
}


void fsm::finish_init(){
  canzero_set_state(levitation_state_IDLE);
}

void fsm::update() {
  levitation_state state;
  levitation_state next_state;

  do {
    const Timestamp now = Timestamp::now();
    Duration time_since_last_transition = now - fsm_last_transition;

    levitation_command cmd = error_handling::approve(canzero_get_command());

    state = canzero_get_state();
    switch (state) {
    case levitation_state_INIT:
      next_state = states::init(cmd, time_since_last_transition);
      break;
    case levitation_state_IDLE:
      next_state = states::init(cmd, time_since_last_transition);
      break;
    case levitation_state_ARMING45:
      next_state = states::arming45(cmd, time_since_last_transition);
      break;
    case levitation_state_PRECHARGE:
      next_state = states::precharge(cmd, time_since_last_transition);
      break;
    case levitation_state_READY:
      next_state = states::ready(cmd, time_since_last_transition);
      break;
    case levitation_state_START:
      next_state = states::start(cmd, time_since_last_transition);
      break;
    case levitation_state_CONTROL:
      next_state = states::control(cmd, time_since_last_transition);
      break;
    case levitation_state_STOP:
      next_state = states::stop(cmd, time_since_last_transition);
      break;
    case levitation_state_DISARMING45:
      next_state = states::disarming45(cmd, time_since_last_transition);
      break;
    }

    if (next_state != state) {
      fsm_last_transition = now;
      canzero_set_state(next_state);
      canzero_update_continue(canzero_get_time());
    }
  } while (next_state != state);
}
