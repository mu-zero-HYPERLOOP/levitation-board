#include "canzero/canzero.h"
#include "fsm/states.h"

mlu_state start_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {
    return mlu_state_CONTROL;
}
