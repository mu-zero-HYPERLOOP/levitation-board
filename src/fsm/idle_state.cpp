#include "canzero/canzero.h"
#include "util/timestamp.h"

mlu_state idle_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {

    if (mlu_command_PRECHARGE == cmd) {
        return mlu_state_PRECHARGE;
    } else {
        //Actions
        return mlu_state_IDLE;
    }
}
