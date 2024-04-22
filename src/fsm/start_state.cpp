#include "canzero/canzero.h"
#include "fsm/states.h"

// Invariant: the control is running! and currently starting
mlu_state start_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {
    
    if (true /* START COMPLETE */ ) {
      return mlu_state_CONTROL;
    }

    // TODO set sdc

    return mlu_state_CONTROL;
}
