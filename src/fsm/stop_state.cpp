#include "canzero/canzero.h"
#include "fsm/states.h"


// Invariant: the control is running! and currently stopping
mlu_state stop_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {
    return mlu_state_READY;
}
