#include "control.h"
#include "airgap_transition.h"
#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "print.h"
#include "sdc_brake.h"
#include "util/boxcar.h"
#include "util/ema.h"
#include "util/metrics.h"
#include "util/timimg.h"
#include <algorithm>
#include <avr/pgmspace.h>
#include <cassert>
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
  float o_max;
  filter_mode error_filter_mode;
  filter_mode conv_filter_mode;
};

struct PID_state {
  float last_error;

  float p_term = 0.0f;
  float i_term = 0.0f;
  float d_term = 0.0f;
  float state_deriviative = 0.0f;
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

static Voltage vdc_voltage;
static float delta_time;

static IntervalTiming isr_interval_timer;

// Left airgap PID
static PID_parameters pid_left_airgap_parameters;
static PID_state pid_left_airgap_state;
static DynamicBoxcar<float> pid_left_airgap_boxcar{0.0f, 200};
static ExponentialMovingAverage<float> pid_left_airgap_ema{};
static ExponentialMovingAverage<Distance> pid_left_airgap_conv_ema{};
static DynamicBoxcar<Distance> pid_left_airgap_conv_boxcar{0_mm, 1};

// Right airgap PID
static PID_parameters pid_right_airgap_parameters;
static PID_state pid_right_airgap_state;
static DynamicBoxcar<float> pid_right_airgap_boxcar{0.0f, 200};
static ExponentialMovingAverage<float> pid_right_airgap_ema{};
static ExponentialMovingAverage<Distance> pid_right_airgap_conv_ema{};
static DynamicBoxcar<Distance> pid_right_airgap_conv_boxcar{0_mm, 1};

// Left current PI
static PI_parameters pi_left_current_parameters;
static PI_state pi_left_current_state;
static ExponentialMovingAverage<Current> pi_left_current_filter{};

// Right current PI
static PI_parameters pi_right_current_parameters;
static PI_state pi_right_current_state;
static ExponentialMovingAverage<Current> pi_right_current_filter{};

// Output debug!
static Voltage debug_left_voltage = 0_V;
static Voltage debug_right_voltage = 0_V;
static Current debug_left_target_current = 0_A;
static Current debug_right_target_current = 0_A;
static Force debug_left_target_force = 0_N;
static Force debug_right_target_force = 0_N;
static Distance debug_left_airgap = 0_mm;
static Distance debug_right_airgap = 0_mm;

static ExponentialMovingAverage<float> debug_left_error2{0.0001};
static ExponentialMovingAverage<float> debug_right_error2{0.0001};

// Error handling
static Timestamp last_airgap_left_ok;
static Timestamp last_airgap_right_ok;
static Distance airgap_error_thresh = 3.5_mm;
static Duration airgap_timeout = 4_ms;

static Timestamp last_current_left_ok;
static Timestamp last_current_right_ok;
static Current current_error_thresh = 30_A;
static Duration current_timeout = 3_s;

void control::begin() {
  const pid_parameters airgap_pid = {
      .m_Kp = 30.0f,
      .m_Ki = 100.0f,
      .m_Kd = 1.8f,
  };
  canzero_set_airgap_pid(airgap_pid);
  const pid_parameters_extra airgap_pid_extra = {
      .m_Ki_min = 0.0f,
      .m_Ki_max = 550.0f,
      .m_force_max = 650.0f,
      .m_filter_mode = filter_mode_BOXCAR,
      .m_boxcar_n = 50,
      .m_ema_alpha = 0.03975,
      .m_conv_filter_mode = filter_mode_BOXCAR,
      .m_conv_boxcar_n = 50,
      .m_conv_ema_alpha = 1,
  };
  canzero_set_airgap_pid_extra(airgap_pid_extra);
  const pi_parameters current_pi = {
      .m_Kp = 20.0f,
      .m_Ki = 10.0f,
  };
  canzero_set_current_pi(current_pi);
  const pi_parameters_extra current_pi_extra = {
      .m_Ki_min = -40.0f,
      .m_Ki_max = 40.0f,
      .m_ema_alpha = 0.5f,
  };
  canzero_set_current_pi_extra(current_pi_extra);

  // reset also updates the globals!
  reset();
}

GuidancePwmControl FASTRUN control::control_loop(Current current_left,
                                                 Current current_right,
                                                 Distance magnet_airgap_left,
                                                 Distance magnet_airgap_right) {
  isr_interval_timer.tick();

  // ====================== ERROR-HANDLING =====================

  const auto now = Timestamp::now();
  if (magnet_airgap_left >= airgap_error_thresh) {
    last_airgap_left_ok = now;
  }
  if (now - last_airgap_left_ok > airgap_timeout) {
   debugPrintf("airgap left fault\n");
   sdc_brake::brake_immediatly();
   return GuidancePwmControl();
  }
  if (magnet_airgap_right >= airgap_error_thresh) {
    last_airgap_right_ok = now;
  }
  if (now - last_airgap_right_ok > airgap_timeout) {
   debugPrintf("airgap right fault\n");
   sdc_brake::brake_immediatly();
   return GuidancePwmControl();
  }
  
  if (current_left <= current_error_thresh) {
    last_current_left_ok = now;
  }
  if (now - last_current_left_ok > current_timeout) {
    sdc_brake::brake_immediatly();
    return GuidancePwmControl();
  }
  if (current_right <= current_error_thresh) {
    last_current_right_ok = now;
  }
  if (now - last_current_right_ok > current_timeout) {
    sdc_brake::brake_immediatly();
    return GuidancePwmControl();
  }

  // ====================== AIRGAP PIDs =========================

  // Left airgap PID
  {
    const Distance target = airgap_transition::current_left();
    // error in mm!
    float filtered_airgap_pid;
    switch (pid_left_airgap_parameters.error_filter_mode) {
    case filter_mode_EMA:
      pid_left_airgap_ema.push(static_cast<float>(magnet_airgap_left));
      filtered_airgap_pid = pid_left_airgap_ema.get();
      break;
    case filter_mode_BOXCAR:
      pid_left_airgap_boxcar.push(static_cast<float>(magnet_airgap_left));
      filtered_airgap_pid = pid_left_airgap_boxcar.get();
      break;
    default:
      filtered_airgap_pid = 0.0f;
    }
    debug_left_airgap = Distance(filtered_airgap_pid);
    float error = (filtered_airgap_pid - static_cast<float>(target)) * 1e3;
    debug_left_error2.push(error);

    pid_left_airgap_state.p_term = error * pid_left_airgap_parameters.Kp;

    pid_left_airgap_state.i_term += error * pid_left_airgap_parameters.Ki /
                                    static_cast<float>(pwm::frequency());
    pid_left_airgap_state.i_term = std::clamp(pid_left_airgap_state.i_term,
                                              pid_left_airgap_parameters.i_min,
                                              pid_left_airgap_parameters.i_max);

    pid_left_airgap_state.d_term = (error - pid_left_airgap_state.last_error) *
                                   static_cast<float>(pwm::frequency()) *
                                   pid_left_airgap_parameters.Kd;
    pid_left_airgap_state.last_error = error;
  }

  const float left_airgap_pid_output =
      std::clamp(pid_left_airgap_state.p_term + pid_left_airgap_state.i_term +
                     pid_left_airgap_state.d_term,
                 0.0f, pid_left_airgap_parameters.o_max);
  debug_left_target_force = Force(left_airgap_pid_output);

  // Right airgap PID
  {
    const Distance target = airgap_transition::current_right();
    // error in mm!
    float filtered_airgap_pid;
    switch (pid_right_airgap_parameters.error_filter_mode) {
    case filter_mode_EMA:
      pid_right_airgap_ema.push(static_cast<float>(magnet_airgap_right));
      filtered_airgap_pid = pid_right_airgap_ema.get();
      break;
    case filter_mode_BOXCAR:
      pid_right_airgap_boxcar.push(static_cast<float>(magnet_airgap_right));
      filtered_airgap_pid = pid_right_airgap_boxcar.get();
      break;
    default:
      filtered_airgap_pid = 0.0f;
    }
    debug_right_airgap = Distance(filtered_airgap_pid);
    float error = (filtered_airgap_pid - static_cast<float>(target)) * 1e3;
    debug_right_error2.push(error);

    pid_right_airgap_state.p_term = error * pid_right_airgap_parameters.Kp;

    pid_right_airgap_state.i_term += error * pid_right_airgap_parameters.Ki /
                                     static_cast<float>(pwm::frequency());
    pid_right_airgap_state.i_term = std::clamp(
        pid_right_airgap_state.i_term, pid_right_airgap_parameters.i_min,
        pid_right_airgap_parameters.i_max);

    pid_right_airgap_state.d_term =
        (error - pid_right_airgap_state.last_error) *
        static_cast<float>(pwm::frequency()) * pid_right_airgap_parameters.Kd;

    pid_right_airgap_state.last_error = error;
  }

  const float right_airgap_pid_output =
      std::clamp(pid_right_airgap_state.p_term + pid_right_airgap_state.i_term +
                     pid_right_airgap_state.d_term,
                 0.0f, pid_right_airgap_parameters.o_max);

  debug_right_target_force = Force(right_airgap_pid_output);

  // ==================== FORCE CURRENT CONVERSION ===============

  /* Distance filtered_left_airgap; */
  /* switch (pid_left_airgap_parameters.conv_filter_mode) { */
  /* case filter_mode_EMA: */
  /*   pid_left_airgap_conv_ema.push(magnet_airgap_left); */
  /*   filtered_left_airgap = pid_left_airgap_conv_ema.get(); */
  /*   break; */
  /* case filter_mode_BOXCAR: */
  /*   pid_left_airgap_conv_boxcar.push(magnet_airgap_left); */
  /*   filtered_left_airgap = pid_left_airgap_conv_boxcar.get(); */
  /* default: */
  /*   filtered_left_airgap = 0_mm; */
  /*   break; */
  /* } */
  float left_current_pi_target = signed_sqrt(left_airgap_pid_output) *
                                     64.5497f *
                                     static_cast<float>(debug_left_airgap) +
                                 std::pow(left_airgap_pid_output * 0.0019f, 3);
  left_current_pi_target = std::clamp(left_current_pi_target, 0.0f, 40.0f);

  /* Distance filtered_right_airgap; */
  /* switch (pid_right_airgap_parameters.conv_filter_mode) { */
  /* case filter_mode_EMA: */
  /*   pid_right_airgap_conv_ema.push(magnet_airgap_right); */
  /*   filtered_right_airgap = pid_right_airgap_conv_ema.get(); */
  /*   break; */
  /* case filter_mode_BOXCAR: */
  /*   pid_right_airgap_conv_boxcar.push(magnet_airgap_right); */
  /*   filtered_right_airgap = pid_right_airgap_conv_boxcar.get(); */
  /* default: */
  /*   filtered_right_airgap = 0_mm; */
  /*   break; */
  /* } */

  float right_current_pi_target =
      signed_sqrt(right_airgap_pid_output) * 64.5497f *
          static_cast<float>(debug_right_airgap) +
      std::pow(right_airgap_pid_output * 0.0019f, 3);
  right_current_pi_target = std::clamp(right_current_pi_target, 0.0f, 40.0f);

  // ====================== CURRENT PIDs =========================
  // Left current PI
  {
    const Current target = Current(left_current_pi_target);
    debug_left_target_current = target;
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
    debug_right_target_current = target;
    pi_right_current_filter.push(current_right);
    const Current filtered_current = pi_right_current_filter.get();
    const float error = static_cast<float>(target - filtered_current);

    pi_right_current_state.p_term = error * pi_right_current_parameters.Kp;

    pi_right_current_state.i_term += error * pi_right_current_parameters.Ki;
    pi_right_current_state.i_term = std::clamp(
        pi_right_current_state.i_term, pi_right_current_parameters.i_min,
        pi_right_current_parameters.i_max);
  }
  float right_current_pi_output =
      pi_right_current_state.p_term + pi_right_current_state.i_term;

  // =============== OUTPUT ==============

  constexpr float CONTROL_LOWER_BOUND = -0.9f;
  constexpr float CONTROL_UPPER_BOUND = 0.9f;

  const Voltage vdc = Voltage(canzero_get_vdc_voltage());

  // clamping here (just for debugging!) remove be on release!
  left_current_pi_output = std::clamp(
      left_current_pi_output, CONTROL_LOWER_BOUND * static_cast<float>(vdc),
      CONTROL_UPPER_BOUND * static_cast<float>(vdc));
  right_current_pi_output = std::clamp(
      right_current_pi_output, CONTROL_LOWER_BOUND * static_cast<float>(vdc),
      CONTROL_UPPER_BOUND * static_cast<float>(vdc));

  const Voltage voltage_left_magnet = Voltage(left_current_pi_output);
  const Voltage voltage_right_magnet = Voltage(right_current_pi_output);

  debug_left_voltage = voltage_left_magnet;
  debug_right_voltage = voltage_right_magnet;

  float controlLeft = voltage_left_magnet / vdc;
  float controlRight = voltage_right_magnet / vdc;

  controlLeft = std::clamp(controlLeft, -0.9f, 0.9f);
  controlRight = std::clamp(controlRight, -0.9f, 0.9f);

  float dutyLL = 0.5 + controlLeft / 2;
  float dutyLR = 0.5 - controlLeft / 2;
  float dutyRL = 0.5 + controlRight / 2;
  float dutyRR = 0.5 - controlRight / 2;

  GuidancePwmControl pwmControl{};
  pwmControl.left_l = dutyLL;
  pwmControl.left_r = dutyLR;
  pwmControl.right_l = dutyRL;
  pwmControl.right_r = dutyRR;
  return pwmControl;
}

void control::reset() {
  auto _lck = guidance_board::InterruptLock::acquire();
  const pid_parameters airgap_pid = canzero_get_airgap_pid();
  const pid_parameters_extra airgap_pid_extra = canzero_get_airgap_pid_extra();

  debug_left_error2.reset(0);
  debug_right_error2.reset(0);

  delta_time = static_cast<float>(1.0f / pwm::frequency());

  // Initalize Left airgap PID!
  pid_left_airgap_parameters.Kp = airgap_pid.m_Kp;
  pid_left_airgap_parameters.Ki = airgap_pid.m_Ki;
  pid_left_airgap_parameters.Kd = airgap_pid.m_Kd;
  pid_left_airgap_parameters.i_min = airgap_pid_extra.m_Ki_min;
  pid_left_airgap_parameters.i_max = airgap_pid_extra.m_Ki_max;
  pid_left_airgap_parameters.o_max = airgap_pid_extra.m_force_max;
  pid_left_airgap_parameters.error_filter_mode = airgap_pid_extra.m_filter_mode;
  switch (airgap_pid_extra.m_filter_mode) {
  case filter_mode_EMA:
    pid_left_airgap_ema.set_alpha(airgap_pid_extra.m_ema_alpha);
    pid_left_airgap_ema.reset(canzero_get_airgap_left() * 1e-3);
    break;
  case filter_mode_BOXCAR:
    pid_left_airgap_boxcar.reset(canzero_get_airgap_left() * 1e-3,
                                 airgap_pid_extra.m_boxcar_n);
    break;
  }
  pid_left_airgap_parameters.conv_filter_mode =
      airgap_pid_extra.m_conv_filter_mode;
  switch (pid_left_airgap_parameters.conv_filter_mode) {
  case filter_mode_EMA:
    pid_left_airgap_conv_ema.set_alpha(airgap_pid_extra.m_conv_ema_alpha);
    pid_left_airgap_conv_ema.reset(Distance(canzero_get_airgap_left() * 1e-3));
    break;
  case filter_mode_BOXCAR:
    pid_left_airgap_conv_boxcar.reset(
        Distance(canzero_get_airgap_left() * 1e-3),
        airgap_pid_extra.m_conv_boxcar_n);
    break;
  }
  pid_left_airgap_state.p_term = 0;
  pid_left_airgap_state.i_term = 0;
  pid_left_airgap_state.d_term = 0;
  pid_left_airgap_state.last_error = 0;

  // Initalize Right airgap PID!
  pid_right_airgap_parameters.Kp = airgap_pid.m_Kp;
  pid_right_airgap_parameters.Ki = airgap_pid.m_Ki;
  pid_right_airgap_parameters.Kd = airgap_pid.m_Kd;
  pid_right_airgap_parameters.i_min = airgap_pid_extra.m_Ki_min;
  pid_right_airgap_parameters.i_max = airgap_pid_extra.m_Ki_max;
  pid_right_airgap_parameters.o_max = airgap_pid_extra.m_force_max;
  pid_right_airgap_parameters.error_filter_mode =
      airgap_pid_extra.m_filter_mode;
  switch (pid_right_airgap_parameters.error_filter_mode) {
  case filter_mode_EMA:
    pid_right_airgap_ema.set_alpha(airgap_pid_extra.m_ema_alpha);
    pid_right_airgap_ema.reset(canzero_get_airgap_left() * 1e-3);
    
    // pid_right_airgap_ema.set_alpha(airgap_pid_extra.m_ema_alpha);
    // pid_right_airgap_ema.reset(canzero_get_airgap_right() * 1e-3);

    break;
  case filter_mode_BOXCAR:
    pid_right_airgap_boxcar.reset(canzero_get_airgap_right() * 1e-3,
                                  airgap_pid_extra.m_boxcar_n);
    break;
  }
  pid_right_airgap_parameters.conv_filter_mode = airgap_pid_extra.m_conv_filter_mode;
  switch (pid_right_airgap_parameters.conv_filter_mode) {
  case filter_mode_EMA:
    pid_right_airgap_conv_ema.set_alpha(airgap_pid_extra.m_conv_ema_alpha);
    pid_right_airgap_conv_ema.reset(
        Distance(canzero_get_airgap_right() * 1e-3));
    break;
  case filter_mode_BOXCAR:
    pid_right_airgap_conv_boxcar.reset(
        Distance(canzero_get_airgap_right() * 1e-3),
        airgap_pid_extra.m_conv_boxcar_n);
    break;
  }
  pid_right_airgap_state.p_term = 0;
  pid_right_airgap_state.i_term = 0;
  pid_right_airgap_state.d_term = 0;
  pid_right_airgap_state.last_error = 0;
  const pi_parameters current_pi = canzero_get_current_pi();
  const pi_parameters_extra current_pi_extra = canzero_get_current_pi_extra();

  // Initalize Left current PI
  pi_left_current_parameters.Kp = current_pi.m_Kp;
  pi_left_current_parameters.Ki = current_pi.m_Ki;
  pi_left_current_parameters.i_min = current_pi_extra.m_Ki_min;
  pi_left_current_parameters.i_max = current_pi_extra.m_Ki_max;
  pi_left_current_state.i_term = 0;
  pi_left_current_state.p_term = 0;
  pi_left_current_filter.set_alpha(current_pi_extra.m_ema_alpha);
  pi_left_current_filter.reset(Current(canzero_get_current_left()));

  // Initalize Right current PI
  pi_right_current_parameters.Kp = current_pi.m_Kp;
  pi_right_current_parameters.Ki = current_pi.m_Ki;
  pi_right_current_parameters.i_min = current_pi_extra.m_Ki_min;
  pi_right_current_parameters.i_max = current_pi_extra.m_Ki_max;
  pi_right_current_state.p_term = 0;
  pi_right_current_state.i_term = 0;
  pi_right_current_filter.set_alpha(current_pi_extra.m_ema_alpha);
  pi_right_current_filter.reset(Current(canzero_get_current_right()));

  const auto now = Timestamp::now();
  last_airgap_left_ok = now;
  last_airgap_right_ok = now;

  last_current_left_ok = now;
  last_current_right_ok = now;
}

void FASTRUN control::update() {

  canzero_set_left_airgap_controller_airgap(debug_left_airgap / 1_mm);
  canzero_set_left_airgap_controller_p_term(pid_left_airgap_state.p_term);
  canzero_set_left_airgap_controller_i_term(pid_left_airgap_state.i_term);
  canzero_set_left_airgap_controller_d_term(pid_left_airgap_state.d_term);
  canzero_set_left_airgap_controller_output(debug_left_target_force / 1_N);
  canzero_set_left_airgap_controller_error2(debug_left_error2.get());

  canzero_set_right_airgap_controller_airgap(debug_right_airgap / 1_mm);
  canzero_set_right_airgap_controller_p_term(pid_right_airgap_state.p_term);
  canzero_set_right_airgap_controller_i_term(pid_right_airgap_state.i_term);
  canzero_set_right_airgap_controller_d_term(pid_right_airgap_state.d_term);
  canzero_set_right_airgap_controller_output(debug_right_target_force / 1_N);
  canzero_set_right_airgap_controller_error2(debug_right_error2.get());

  canzero_set_left_current_controller_target(debug_left_target_current / 1_A);
  canzero_set_left_current_controller_p_term(pi_left_current_state.p_term);
  canzero_set_left_current_controller_i_term(pi_left_current_state.i_term);
  canzero_set_left_current_controller_output(debug_left_voltage / 1_V);

  canzero_set_right_current_controller_target(debug_right_target_current / 1_A);
  canzero_set_right_current_controller_p_term(pi_right_current_state.p_term);
  canzero_set_right_current_controller_i_term(pi_right_current_state.i_term);
  canzero_set_right_current_controller_output(debug_right_voltage / 1_V);

  const Frequency isr_frequency = isr_interval_timer.frequency();

  canzero_set_control_frequency(isr_frequency / 1_kHz);
}
