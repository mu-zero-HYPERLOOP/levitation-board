#pragma once

#include "canzero/canzero.h"
#include "util/timestamp.h"

namespace fsm::states {

levitation_state init(levitation_command cmd,
                                 Duration time_since_last_transition);

levitation_state idle(levitation_command cmd,
                                 Duration time_since_last_transition);

levitation_state arming45(levitation_command cmd,
                                      Duration time_since_last_transition);

levitation_state precharge(levitation_command cmd,
                                      Duration time_since_last_transition);

levitation_state ready(levitation_command cmd,
                                  Duration time_since_last_transition);

levitation_state start(levitation_command cmd,
                                  Duration time_since_last_transition);

levitation_state control(levitation_command cmd,
                                    Duration time_since_last_transition);

levitation_state stop(levitation_command cmd,
                                 Duration time_since_last_transition);

levitation_state disarming45(levitation_command cmd,
                                 Duration time_since_last_transition);

}
