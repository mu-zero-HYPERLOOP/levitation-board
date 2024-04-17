#include "canzero.h"
#include "states.h"

mlu_state control_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {
    return mlu_state_READY;
}
