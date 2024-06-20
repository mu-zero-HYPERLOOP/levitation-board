#pragma once

#include "util/metrics.h"
namespace sensors::magnet_current {

constexpr Frequency MEAS_FREQUENCY = 918_Hz;
constexpr float SENSE_GAIN = 20.0f;

void begin();

void calibrate();

void update();

}
