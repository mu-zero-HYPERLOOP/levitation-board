#include "canzero/canzero.h"
#include "fsm/states.h"

mlu_state precharge_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {
    // TODO what happens on disconnect
    // TODO what happens when we remain to long in this state?

    if (true /* TODO PRECHARGE-DONE */) {
        return mlu_state_READY; 
    } 

    // TODO set sdc
    return mlu_state_PRECHARGE;
}
