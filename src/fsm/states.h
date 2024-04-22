#pragma once

#include "canzero/canzero.h"
#include "util/timestamp.h"

mlu_state init_state_next(mlu_command cmd, Duration time_since_last_transition);

mlu_state idle_state_next(mlu_command cmd, Duration time_since_last_transition);

mlu_state precharge_state_next(mlu_command cmd, Duration time_since_last_transition);

mlu_state ready_state_next(mlu_command cmd, Duration time_since_last_transition);

mlu_state start_state_next(mlu_command cmd, Duration time_since_last_transition);

mlu_state control_state_next(mlu_command cmd, Duration time_since_last_transition);

mlu_state stop_state_next(mlu_command cmd, Duration time_since_last_transition);
