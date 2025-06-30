
#include "firmware/pwm.h"

struct GuidancePwmControl {
  float left_l = 0.5;  // pwm (22).
  float left_r = 0.5;  // pwm (42)
  float right_l = 0.5; // pwm (20)
  float right_r = 0.5; // pwm (13)
  constexpr operator PwmControl() const {
    return PwmControl(right_l, left_l, 0.0f, left_r, 0.0f, right_r);
  }

  GuidancePwmControl() : left_l(0.5), left_r(0.5), right_l(0.5), right_r(0.5) {}
};
