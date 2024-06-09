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

struct PID {
  float m_dt;
  float m_integral;
  float m_last_error;
  float m_Kp;
  float m_Ki;
  float m_Kd;

  float m_max_i;
  float m_min_i;

  float m_p_term = 0.0f;
  float m_i_term = 0.0f;
  float m_d_term = 0.0f;

  float step(float error) {
    m_integral += error * m_dt;
    m_integral = std::clamp(m_integral, m_min_i, m_max_i);
    float derivitive = (error - m_last_error) / m_dt;
    m_last_error = error;
    m_p_term = error * m_Kp;
    m_i_term = m_integral * m_Ki;
    m_d_term = derivitive * m_Kd;
    return m_p_term + m_i_term + m_d_term;
  }
};

struct PI {

  float m_dt;
  float m_integral;
  float m_Kp;
  float m_Ki;

  float m_max_i;
  float m_min_i;

  float m_p_term = 0.0f;
  float m_i_term = 0.0f;

  float step(float error) {
    m_integral += error * m_dt;
    m_integral = std::clamp(m_integral, m_min_i, m_max_i);
    m_p_term = error * m_Kp;
    m_i_term = m_integral * m_Ki;
    return m_p_term + m_i_term;
  }
};

static PID left_airgap;
static PID right_airgap;
static PI left_current;
static PI right_current;

static ExponentialMovingAverage left_error_current_filter(0.01);
static ExponentialMovingAverage right_error_current_filter(0.01);

static ExponentialMovingAverage<Distance> left_error_airgap_filter(0.01, 0_m);
static ExponentialMovingAverage<Distance> right_error_airgap_filter(0.01, 0_m);

void control::begin() {
  const pid_parameters airgap_pid = {
      .m_Kp = 0.0f,
      .m_Ki = 0.0f,
      .m_Kd = 0.0f,
  };
  canzero_set_airgap_pid(airgap_pid);
  left_airgap.m_dt = static_cast<float>(1.0f / pwm::frequency());
  left_airgap.m_integral = 0.0;
  left_airgap.m_last_error = 0.0f;
  left_airgap.m_Kp = airgap_pid.m_Kp;
  left_airgap.m_Ki = airgap_pid.m_Ki;
  left_airgap.m_Kd = airgap_pid.m_Kd;
  left_airgap.m_min_i = -4.0f;
  left_airgap.m_max_i = 50.0f;


  right_airgap.m_dt = static_cast<float>(1.0f / pwm::frequency());
  right_airgap.m_integral = 0.0f;
  right_airgap.m_last_error = 0.0f;
  right_airgap.m_Kp = airgap_pid.m_Kp;
  right_airgap.m_Ki = airgap_pid.m_Ki;
  right_airgap.m_Kd = airgap_pid.m_Kd;
  right_airgap.m_min_i = -4.0f;
  right_airgap.m_max_i = 50.0f;


  const pid_parameters current_pid = {
      .m_Kp = 0.0f,
      .m_Ki = 0.0f,
      .m_Kd = 0.0f,
  };
  canzero_set_current_pi(current_pid);
  left_current.m_dt = static_cast<float>(1.0f / pwm::frequency());
  left_current.m_integral = 0.0f;
  left_current.m_Kp = current_pid.m_Kp;
  left_current.m_Ki = current_pid.m_Ki;
  left_current.m_min_i = -10.0f;
  left_current.m_max_i = 10.0f;

  right_current.m_dt = static_cast<float>(1.0f / pwm::frequency());
  right_current.m_integral = 0.0f;
  right_current.m_Kp = current_pid.m_Kp;
  right_current.m_Ki = current_pid.m_Ki;
  right_current.m_min_i = -10.0f;
  right_current.m_max_i = 10.0f;

}

GuidancePwmControl FASTRUN control::control_loop(Current current_left,
                                                 Current current_right,
                                                 Distance magnet_airgap_left,
                                                 Distance magnet_airgap_right) {
  debugPrintf("HI\n");

  // ====================== AIRGAP PIDs =========================
  const Distance target_airgap_left = airgap_transition::current_left();
  const Distance airgap_left_error = target_airgap_left - magnet_airgap_left;
  left_error_airgap_filter.push(airgap_left_error);
  const float left_airgap_pid_output =
      left_airgap.step(static_cast<float>(left_error_airgap_filter.get()));

  const Distance target_airgap_right = airgap_transition::current_right();
  const Distance airgap_right_error = target_airgap_right - magnet_airgap_right;
  right_error_airgap_filter.push(airgap_right_error);
  const float right_airgap_pid_output =
      right_airgap.step(static_cast<float>(right_error_airgap_filter.get()));

  // ======================= CURRENT PIDs =======================
  const float left_current_error =
      left_airgap_pid_output - static_cast<float>(current_left);
  left_error_current_filter.push(left_current_error);
  const float left_current_pi_output =
      left_current.step(left_error_current_filter.get());

  const float right_current_error =
      right_airgap_pid_output - static_cast<float>(current_left);
  right_error_current_filter.push(right_current_error);
  const float right_current_pi_output =
      right_current.step(right_error_current_filter.get());

  const Voltage voltage_left_magnet = Voltage(left_current_pi_output);
  const Voltage voltage_right_magnet = Voltage(right_current_pi_output);

  // TODO use vdc voltage reading
  float controlLeft = voltage_left_magnet / 45.0_V;
  float controlRight = voltage_right_magnet / 45.0_V;

  controlLeft = std::clamp(controlLeft, -0.2f, 0.8f);
  controlRight = std::clamp(controlRight, -0.2f, 0.8f);

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


void control::reset(){
  auto _lck = guidance_board::InterruptLock::acquire();
  const pid_parameters airgap_pid = canzero_get_airgap_pid();
  left_airgap.m_dt = static_cast<float>(1.0f / pwm::frequency());
  left_airgap.m_integral = 0.0;
  left_airgap.m_last_error = 0.0f;
  left_airgap.m_Kp = airgap_pid.m_Kp;
  left_airgap.m_Ki = airgap_pid.m_Ki;
  left_airgap.m_Kd = airgap_pid.m_Kd;
  left_airgap.m_min_i = -4.0f;
  left_airgap.m_max_i = 50.0f;
  

  right_airgap.m_dt = static_cast<float>(1.0f / pwm::frequency());
  right_airgap.m_integral = 0.0f;
  right_airgap.m_last_error = 0.0f;
  right_airgap.m_Kp = airgap_pid.m_Kp;
  right_airgap.m_Ki = airgap_pid.m_Ki;
  right_airgap.m_Kd = airgap_pid.m_Kd;
  right_airgap.m_min_i = -4.0f;
  right_airgap.m_max_i = 50.0f;


  const pid_parameters current_pid = canzero_get_current_pi();
  left_current.m_dt = static_cast<float>(1.0f / pwm::frequency());
  left_current.m_integral = 0.0f;
  left_current.m_Kp = current_pid.m_Kp;
  left_current.m_Ki = current_pid.m_Ki;
  left_current.m_min_i = -10.0f;
  left_current.m_max_i = 10.0f;

  right_current.m_dt = static_cast<float>(1.0f / pwm::frequency());
  right_current.m_integral = 0.0f;
  right_current.m_Kp = current_pid.m_Kp;
  right_current.m_Ki = current_pid.m_Ki;
  right_current.m_min_i = -10.0f;
  right_current.m_max_i = 10.0f;
}

void FASTRUN control::update() {

  canzero_set_left_airgap_controller_p_term(left_airgap.m_p_term);
  canzero_set_left_airgap_controller_i_term(left_airgap.m_i_term);
  canzero_set_left_airgap_controller_d_term(left_airgap.m_d_term);

  canzero_set_right_airgap_controller_p_term(right_airgap.m_p_term);
  canzero_set_right_airgap_controller_i_term(right_airgap.m_i_term);
  canzero_set_right_airgap_controller_d_term(right_airgap.m_d_term);

  canzero_set_left_current_controller_p_term(left_current.m_p_term);
  canzero_set_left_current_controller_i_term(left_current.m_i_term);

  canzero_set_right_current_controller_p_term(right_current.m_p_term);
  canzero_set_right_current_controller_i_term(right_current.m_i_term);
  
}