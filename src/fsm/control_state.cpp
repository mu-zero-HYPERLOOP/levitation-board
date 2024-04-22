#include "canzero/canzero.h"
#include "fsm/states.h"

// Invariant : control is running!
mlu_state control_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {
    return mlu_state_READY;
}
