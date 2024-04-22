#include "canzero/canzero.h"

#include "fsm/error_handling.h"
#include "fsm/states.h"
#include "util/timestamp.h"

Timestamp g_fsm_last_transition = Timestamp::now();

void fsm_init() {
  g_fsm_last_transition = Timestamp::now();
  canzero_set_state(mlu_state_INIT);
}

void fsm_next() {
    Timestamp now = Timestamp::now();
    Duration time_since_last_transition = now - g_fsm_last_transition;

    mlu_state state = canzero_get_state();
    mlu_command cmd =
        handle_errors(state, canzero_get_command(), time_since_last_transition);

    mlu_state next_state;
    switch(state) {
        case mlu_state_INIT:
            next_state = init_state_next(cmd, time_since_last_transition);
            break;
        case mlu_state_IDLE:
            next_state = idle_state_next(cmd, time_since_last_transition);
            break;
        case mlu_state_PRECHARGE:
            next_state = precharge_state_next(cmd, time_since_last_transition);
            break;
        case mlu_state_READY:
            next_state = ready_state_next(cmd, time_since_last_transition);
            break;
        case mlu_state_START:
            next_state = start_state_next(cmd, time_since_last_transition);
            break;
        case mlu_state_CONTROL:
            next_state = control_state_next(cmd, time_since_last_transition);
            break;
        case mlu_state_STOP:
            next_state = stop_state_next(cmd, time_since_last_transition);
            break;
    }

    if (next_state != state) {
        g_fsm_last_transition = now;
        canzero_set_state(next_state);
    }
}
