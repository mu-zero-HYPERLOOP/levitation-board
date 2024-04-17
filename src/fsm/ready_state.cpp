#include "canzero.h"
#include "states.h"

mlu_state ready_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {

    if (cmd == "TODO") {
        return mlu_state_START;
    } else {
        return mlu_state_READY;
    }
}
