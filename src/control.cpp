#include "control.h"
#include "airgap_transition.h"
#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "print.h"
#include "util/csv.h"
#include "util/ema.h"
#include "util/metrics.h"
#include "util/rectange_integral.h"
#include "util/trapazoidal_integral.h"
#include <algorithm>
#include <avr/pgmspace.h>
#include <cmath>

static float signed_sqrt(float x) {
  if (x < 0) {
    return -std::sqrt(-x);
  } else {
    return std::sqrt(x);
  }
}

struct PID_parameters {
  float Kp;
  float Ki;
  float Kd;

  float i_min;
  float i_max;
};

struct PID_state {
  float last_error;

  float p_term = 0.0f;
  float i_term = 0.0f;
  float d_term = 0.0f;
};

struct PI_parameters {
  float Kp;
  float Ki;

  float i_min;
  float i_max;
};

struct PI_state {
  float p_term = 0.0f;
  float i_term = 0.0f;
};

static float delta_time;

// Left airgap PID
static PID_parameters pid_left_airgap_parameters;
static PID_state pid_left_airgap_state;
static ExponentialMovingAverage<Distance> pid_left_airgap_filter(0.01, 12_mm);

// Right airgap PID
static PID_parameters pid_right_airgap_parameters;
static PID_state pid_right_airgap_state;
static ExponentialMovingAverage<Distance> pid_right_airgap_filter(0.01, 12_mm);

// Left current PI
static PI_parameters pi_left_current_parameters;
static PI_state pi_left_current_state;
static ExponentialMovingAverage<Current> pi_left_current_filter(0.5, 0_A);

// Right current PI
static PI_parameters pi_right_current_parameters;
static PI_state pi_right_current_state;
static ExponentialMovingAverage<Current> pi_right_current_filter(0.5, 0_A);

void control::begin() {
  const pid_parameters airgap_pid = {
      .m_Kp = 1.3f,
      .m_Ki = 0.5f,
      .m_Kd = 0.0f,
  };
  canzero_set_airgap_pid(airgap_pid);

  delta_time = static_cast<float>(1.0f / pwm::frequency());

  // Initalize Left airgap PID!
  pid_left_airgap_parameters.Kp = airgap_pid.m_Kp;
  pid_left_airgap_parameters.Ki = airgap_pid.m_Ki;
  pid_left_airgap_parameters.Kd = airgap_pid.m_Kd;
  pid_left_airgap_parameters.i_min = -4.0f;
  pid_left_airgap_parameters.i_max = 20.0f;
  pid_left_airgap_state.p_term = 0;
  pid_left_airgap_state.i_term = 0;
  pid_left_airgap_state.d_term = 0;

  // Initalize Right airgap PID!
  pid_right_airgap_parameters.Kp = airgap_pid.m_Kp;
  pid_right_airgap_parameters.Ki = airgap_pid.m_Ki;
  pid_right_airgap_parameters.Kd = airgap_pid.m_Kd;
  pid_right_airgap_parameters.i_min = 4.0f;
  pid_right_airgap_parameters.i_max = 20.0f;
  pid_right_airgap_state.p_term = 0;
  pid_right_airgap_state.i_term = 0;
  pid_right_airgap_state.d_term = 0;

  const pid_parameters current_pid = {
      .m_Kp = 5.0f,
      .m_Ki = 0.5f,
      .m_Kd = 0.0f,
  };
  canzero_set_current_pi(current_pid);

  // Initalize Left current PI
  pi_left_current_parameters.Kp = current_pid.m_Kp;
  pi_left_current_parameters.Ki = current_pid.m_Ki;
  pi_left_current_parameters.i_min = -40.0f;
  pi_left_current_parameters.i_max = 40.0f;
  pi_left_current_state.i_term = 0;
  pi_left_current_state.p_term = 0;

  // Initalize Right current PI
  pi_right_current_parameters.Kp = current_pid.m_Kp;
  pi_right_current_parameters.Ki = current_pid.m_Ki;
  pi_right_current_parameters.i_min = -40.0f;
  pi_right_current_parameters.i_max = 40.0f;
  pi_right_current_state.p_term = 0;
  pi_right_current_state.i_term = 0;
}

GuidancePwmControl FASTRUN control::control_loop(Current current_left,
                                                 Current current_right,
                                                 Distance magnet_airgap_left,
                                                 Distance magnet_airgap_right) {

  debugPrintf("current right %f\n", static_cast<float>(current_right));
  debugPrintf("airgap  right %f\n", static_cast<float>(magnet_airgap_right));

  // ====================== AIRGAP PIDs =========================

  // Left airgap PID
  {
    const Distance target = airgap_transition::current_left();
    pid_left_airgap_filter.push(magnet_airgap_left);
    const Distance filtered_airgap = pid_left_airgap_filter.get();
    const float error = static_cast<float>(target - filtered_airgap);

    pid_left_airgap_state.p_term = error * pid_left_airgap_parameters.Kp;

    pid_left_airgap_state.i_term +=
        error * pid_left_airgap_parameters.Ki;
    pid_left_airgap_state.i_term = std::clamp(pid_left_airgap_state.i_term,
                                              pid_left_airgap_parameters.i_min,
                                              pid_left_airgap_parameters.i_max);

    pid_left_airgap_state.d_term =
        ((pid_left_airgap_state.last_error - error) / delta_time) *
        pid_left_airgap_parameters.Kd;
  }
  const float left_airgap_pid_output = pid_left_airgap_state.p_term +
                                       pid_left_airgap_state.i_term +
                                       pid_left_airgap_state.d_term;

  // Right airgap PID
  {
    const Distance target = airgap_transition::current_right();
    pid_right_airgap_filter.push(magnet_airgap_right);
    const Distance filtered_airgap = pid_right_airgap_filter.get();
    const float error = static_cast<float>(target - filtered_airgap);

    pid_right_airgap_state.p_term = error * pid_right_airgap_parameters.Kp;

    pid_right_airgap_state.i_term +=
        error  * pid_right_airgap_parameters.Ki;
    pid_right_airgap_state.i_term = std::clamp(
        pid_right_airgap_state.i_term, pid_right_airgap_parameters.i_min,
        pid_right_airgap_parameters.i_max);

    pid_right_airgap_state.d_term =
        ((pid_right_airgap_state.last_error - error) / delta_time) *
        pid_right_airgap_parameters.Kd;
  }
  const float right_airgap_pid_output = pid_right_airgap_state.p_term +
                                        pid_right_airgap_state.i_term +
                                        pid_right_airgap_state.d_term;

  // ==================== FORCE CURRENT CONVERSION ===============
  /* const float left_current_pi_target = signed_sqrt(left_airgap_pid_output);
   */
  /* const float right_current_pi_target = signed_sqrt(right_airgap_pid_output);
   */
  const float left_current_pi_target = 5;
  const float right_current_pi_target = 5;

  // ====================== CURRENT PIDs =========================
  // Left current PI
  {
    const Current target = Current(left_current_pi_target);
    pi_left_current_filter.push(current_left);
    const Current filtered_current = pi_left_current_filter.get();
    const float error = static_cast<float>(target - filtered_current);

    pi_left_current_state.p_term = error * pi_left_current_parameters.Kp;

    pi_left_current_state.i_term += error * pi_left_current_parameters.Ki;
    pi_left_current_state.i_term = std::clamp(pi_left_current_state.i_term,
                                              pi_left_current_parameters.i_min,
                                              pi_left_current_parameters.i_max);
  }
  float left_current_pi_output =
      pi_left_current_state.p_term + pi_left_current_state.i_term;

  // Right current PI
  {
    const Current target = Current(right_current_pi_target);
    pi_right_current_filter.push(current_right);
    const Current filtered_current = pi_right_current_filter.get();
    debugPrintf("filtered_current %f\n", static_cast<float>(filtered_current));
    const float error = static_cast<float>(target - filtered_current);

    pi_right_current_state.p_term = error * pi_right_current_parameters.Kp;

    pi_right_current_state.i_term += error * pi_right_current_parameters.Ki;
    pi_right_current_state.i_term = std::clamp(
        pi_right_current_state.i_term, pi_right_current_parameters.i_min,
        pi_right_current_parameters.i_max);
  }
  float right_current_pi_output =
      pi_right_current_state.p_term + pi_right_current_state.i_term;

  debugPrintf("out voltage = %f\n", right_current_pi_output);
  debugPrintFlush();

  // =============== OUTPUT ==============

  const Voltage voltage_left_magnet = Voltage(left_current_pi_output);
  const Voltage voltage_right_magnet = Voltage(right_current_pi_output);

  // TODO use vdc voltage reading
  float controlLeft = voltage_left_magnet / 45.0_V;
  float controlRight = voltage_right_magnet / 45.0_V;

  controlLeft = std::clamp(controlLeft, -0.9f, 0.9f);
  controlRight = std::clamp(controlRight, -0.9f, 0.9f);

  const float dutyLL = 0.5 + controlLeft / 2;
  const float dutyLR = 0.5 - controlLeft / 2;
  const float dutyRL = 0.5 + controlRight / 2;
  const float dutyRR = 0.5 - controlRight / 2;

  GuidancePwmControl pwmControl{};
  pwmControl.left_l = dutyLL;
  pwmControl.left_r = dutyLR;
  pwmControl.right_l = dutyRL;
  pwmControl.right_r = dutyRR;
  return pwmControl;
}

void control::reset() {
  return;
  auto _lck = guidance_board::InterruptLock::acquire();
  const pid_parameters airgap_pid = canzero_get_airgap_pid();

  delta_time = static_cast<float>(1.0f / pwm::frequency());

  // Initalize Left airgap PID!
  pid_left_airgap_parameters.Kp = airgap_pid.m_Kp;
  pid_left_airgap_parameters.Ki = airgap_pid.m_Ki;
  pid_left_airgap_parameters.Kd = airgap_pid.m_Kd;
  pid_left_airgap_state.p_term = 0;
  pid_left_airgap_state.i_term = 0;
  pid_left_airgap_state.d_term = 0;

  // Initalize Right airgap PID!
  pid_right_airgap_parameters.Kp = airgap_pid.m_Kp;
  pid_right_airgap_parameters.Ki = airgap_pid.m_Ki;
  pid_right_airgap_parameters.Kd = airgap_pid.m_Kd;
  pid_right_airgap_state.p_term = 0;
  pid_right_airgap_state.i_term = 0;
  pid_right_airgap_state.d_term = 0;

  const pid_parameters current_pid = canzero_get_current_pi();

  // Initalize Left current PI
  pi_left_current_parameters.Kp = current_pid.m_Kp;
  pi_left_current_parameters.Ki = current_pid.m_Ki;
  pi_right_current_parameters.i_min = -10.0f;
  pi_right_current_parameters.i_max = 10.0f;
  pi_left_current_state.i_term = 0;
  pi_left_current_state.p_term = 0;

  // Initalize Right current PI
  pi_right_current_parameters.Kp = current_pid.m_Kp;
  pi_right_current_parameters.Ki = current_pid.m_Ki;
  pi_right_current_parameters.i_min = -10.0f;
  pi_right_current_parameters.i_max = 10.0f;
  pi_right_current_state.p_term = 0;
  pi_right_current_state.i_term = 0;
}

void FASTRUN control::update() {

  canzero_set_left_airgap_controller_p_term(pid_left_airgap_state.p_term);
  canzero_set_left_airgap_controller_i_term(pid_left_airgap_state.i_term);
  canzero_set_left_airgap_controller_d_term(pid_left_airgap_state.d_term);

  canzero_set_right_airgap_controller_p_term(pid_right_airgap_state.p_term);
  canzero_set_right_airgap_controller_i_term(pid_right_airgap_state.i_term);
  canzero_set_right_airgap_controller_d_term(pid_right_airgap_state.d_term);

  canzero_set_left_current_controller_p_term(pi_left_current_state.p_term);
  canzero_set_left_current_controller_i_term(pi_left_current_state.i_term);

  canzero_set_right_current_controller_p_term(pi_right_current_state.p_term);
  canzero_set_right_current_controller_i_term(pi_right_current_state.i_term);
}
