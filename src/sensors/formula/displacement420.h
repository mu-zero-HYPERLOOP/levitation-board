#pragma once

#include "util/metrics.h"

namespace sensors::formula {

static constexpr inline Distance displacement420(Current i) {
  return (i - 4_mA) * (50_mm - 25_mm) / 16_mA + 25_mm;
}

static constexpr Current inv_displacement420(Distance d) {
  return ((d - 20_mm) * 16_mA) / (50_mm - 20_mm) + 4_mA;
}

} // namespace sensors::formula
