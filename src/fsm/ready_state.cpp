#include "canzero/canzero.h"
#include "fsm/states.h"

mlu_state ready_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {
    
    // what happens on disconnect or start

    // TODO set sdc
    

    return mlu_state_READY;
}
