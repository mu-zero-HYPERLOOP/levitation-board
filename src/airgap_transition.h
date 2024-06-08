#pragma once

#include "util/metrics.h"
namespace airgap_transition {

void begin();

void calibrate();

void transition_to_ground(const Time& duration);

void transition_to(const Distance& distance, const Time& duration);

bool done();

void update();

}
