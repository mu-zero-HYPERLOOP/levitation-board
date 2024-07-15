#pragma once

#include "util/metrics.h"
namespace sensors::magnet_temperatures {


constexpr Frequency MEAS_FREQUENCY = 100_Hz;

constexpr Resistance R_MEAS = 99_Ohm;
constexpr Resistance PTX_R0_L = 100_Ohm;
constexpr Resistance PTX_R0_R = 100_Ohm;

void begin();

void calibrate();

void update();

} // namespace sensors::magnet_temperatures
