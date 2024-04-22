#include "canzero/canzero.h"
#include "util/timestamp.h"

mlu_state idle_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {
    // TODO this transition should be from all states of the fsm!
    if (mlu_command_DISCONNECT == cmd) { //btw nice yoda notation =^)
      return mlu_state_INIT;
    }

    if (mlu_command_PRECHARGE == cmd) {
        return mlu_state_PRECHARGE;
    } 
    
    // TODO set sdc

    return mlu_state_IDLE;
}
