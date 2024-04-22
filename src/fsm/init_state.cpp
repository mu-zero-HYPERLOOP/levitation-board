#include "canzero/canzero.h"
#include "fsm/states.h"

mlu_state init_state_next(mlu_command cmd,
                            Duration time_since_last_transition) {
    // TODO initalize IO devices!
    // - xbar
    // - pwm
    // - adcetc.
    // TODO: set SDC
    return mlu_state_IDLE;
}
