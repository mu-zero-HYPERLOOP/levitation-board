#include "canzero/canzero.h"
#include "fsm/states.h"

mlu_state ready_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {

    //TODO
    if (cmd == mlu_command_NONE) {
        return mlu_state_START;
    } else {
        return mlu_state_READY;
    }
}
