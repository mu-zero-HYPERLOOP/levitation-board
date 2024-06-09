#include "control.h"
#include "canzero/canzero.h"
#include "print.h"
#include "util/csv.h"
#include "util/ema.h"
#include "util/metrics.h"
#include "util/trapazoidal_integral.h"
#include <algorithm>
#include <avr/pgmspace.h>
#include <Arduino.h>

void control::begin() {
}

GuidancePwmControl FASTRUN control::control_loop(Current current_left,
                                                 Current current_right,
                                                 Distance magnet_airgap_left,
                                                 Distance magnet_airgap_right) {

  Serial.println("HI");
  return GuidancePwmControl{};

  const Voltage v = 0_V;

  float controlLeft = v / 45.0_V;
  const float controlRight = 0.0f;
  controlLeft = std::clamp(controlLeft, -0.15f, 0.15f);

  float dutyLL = 0.5 + controlLeft / 2;
  float dutyLR = 0.5 - controlLeft / 2;
  float dutyRL = 0.5 + controlRight / 2;
  float dutyRR = 0.5 - controlRight / 2;

  dutyLL = std::clamp(dutyLL, 0.1f, 0.9f);
  dutyLR = std::clamp(dutyLR, 0.1f, 0.9f);
  dutyRL = std::clamp(dutyRL, 0.1f, 0.9f);
  dutyRR = std::clamp(dutyRR, 0.1f, 0.9f);


  GuidancePwmControl pwmControl{};
  pwmControl.left_l = dutyLL;
  pwmControl.left_r = dutyLR;
  pwmControl.right_l = dutyRL;
  pwmControl.right_r = dutyRR;
  return pwmControl;
}

void FASTRUN control::update() { 
}
