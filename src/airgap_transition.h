#pragma once

#include "util/metrics.h"
#include "util/timestamp.h"
namespace airgap_transition {

void begin();

void calibrate();

void transition_to_ground(const Duration& duration);

void transition_to(const Distance& distance, const Duration& duration);

// this method can be called from a ISR.
Distance current_left();

// this method can be called from a ISR.
Distance current_right();

bool done();

void update();

}
