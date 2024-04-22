#pragma once

#include "canzero/canzero.h"
#include "util/timestamp.h"

mlu_command handle_errors(mlu_state state, mlu_command cmd, Duration time_since_last_transition);
