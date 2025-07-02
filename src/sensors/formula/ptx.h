#pragma once

#include "util/metrics.h"
namespace sensors::formula {


static constexpr float ALPHA = 3.85e-3;

inline Temperature ptx(Resistance r_ptx, Resistance r0) {
  return 0_Celcius + Temperature((r_ptx - r0) / (r0 * ALPHA));
}

inline Resistance inv_ptx(Temperature temp, Resistance r0) {
  return static_cast<float>(temp - 0_Celcius) * r0 * ALPHA + r0;
}

}
