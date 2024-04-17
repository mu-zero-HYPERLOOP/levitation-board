#pragma once

#include "canzero.h"
#include "timestamp.h"

mlu_command handle_errors(mlu_state state, mlu_command cmd, Duration time_since_last_transition);