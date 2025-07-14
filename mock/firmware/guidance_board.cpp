#include "firmware/guidance_board.h"
#include "adc_isr.h"
#include "canzero/canzero.h"
#include "firmware/ain_scheduler.h"
#include "firmware/xbar.h"
#include "print.h"
#include "sensors/airgaps.h"
#include "sensors/formula/current_sense.h"
#include "sensors/formula/displacement420.h"
#include "sensors/formula/isolated_voltage.h"
#include "sensors/formula/ntc_beta.h"
#include "sensors/formula/ptx.h"
#include "sensors/formula/voltage_divider.h"
#include "sensors/input_current.h"
#include "util/boxcar.h"
#include "util/interval.h"
#include "util/metrics.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <random>
#include <thread>

constexpr size_t MAX_AIN_PERIODIC_JOBS = 10;

static AinScheduler<MAX_AIN_PERIODIC_JOBS> ain_scheduler;
static std::random_device rd{};
static std::mt19937 gen{rd()};

static Interval isr_interval(1_min);

void FLASHMEM guidance_board::begin() {
  xbar::begin();
  isr_interval = Interval(pwm::frequency());
}

static Voltage mock_disp(Distance distance) {
  std::normal_distribution dist{static_cast<float>(distance), 10e-6f};
  const Distance d = Distance(dist(gen));
  const Current i = sensors::formula::inv_displacement420(d);
  const Voltage v = i * sensors::airgaps::R_MEAS;
  return v;
}

static Voltage mock_current(Current current, float gain) {
  std::normal_distribution dist{static_cast<float>(current), 0.5f};
  const Current i = Current(dist(gen));
  const Voltage v = sensors::formula::inv_current_sense(i, gain);
  return v;
}

static Voltage mock_temperature(Temperature temp, Resistance ptx_r0,
                                Resistance r2) {
  std::normal_distribution dist{static_cast<float>(temp), 1.0f};
  const Temperature t = Temperature(dist(gen));
  const Resistance ptx_r = sensors::formula::inv_ptx(t, ptx_r0);
  return sensors::formula::vout_of_voltage_divider(3.3_V, ptx_r, r2);
}

extern PwmControl __pwm_control;

BoxcarFilter<Voltage, 1000> vdc_voltage(0_V);

static float current_left = 0;
static float current_right = 0;

Voltage FASTRUN guidance_board::sync_read(ain_pin pin) {
  switch (pin) {
  case ain_pin::disp_sense_mag_l_19:
  case ain_pin::disp_sense_lim_l_18:
    switch (canzero_get_state()) {
    case levitation_state_INIT:
      return mock_disp(16_mm + 28_mm);
    case levitation_state_IDLE:
    case levitation_state_ARMING45:
    case levitation_state_PRECHARGE:
    case levitation_state_READY:
    case levitation_state_START:
    case levitation_state_CONTROL:
    case levitation_state_STOP:
    case levitation_state_DISARMING45:
      return mock_disp(Distance(canzero_get_target_airgap_left() * 1e-3) +
                       16_mm + 6_mm - 0.5_mm);
    }
  case ain_pin::disp_sense_mag_r_17:
  case ain_pin::disp_sense_lim_r_16:
    switch (canzero_get_state()) {
    case levitation_state_INIT:
      return mock_disp(16_mm + 28_mm);
    case levitation_state_IDLE:
    case levitation_state_ARMING45:
    case levitation_state_PRECHARGE:
    case levitation_state_READY:
    case levitation_state_START:
    case levitation_state_CONTROL:
    case levitation_state_STOP:
    case levitation_state_DISARMING45:
      return mock_disp(Distance(canzero_get_target_airgap_right() * 1e-3) +
                       16_mm + 6_mm - 0.5_mm);
    }
  case ain_pin::temp_sense_l2_21:
  case ain_pin::temp_sense_l1_20:
  case ain_pin::temp_sense_r2_15:
  case ain_pin::temp_sense_r1_14:
  case ain_pin::vdc_sense_40: {
    switch (canzero_get_state()) {
    case levitation_state_INIT:
    case levitation_state_IDLE:
    case levitation_state_ARMING45:
    case levitation_state_DISARMING45:
      vdc_voltage.push(0_V);
      break;
    case levitation_state_PRECHARGE:
    case levitation_state_READY:
    case levitation_state_CONTROL:
    case levitation_state_START:
    case levitation_state_STOP:
      vdc_voltage.push(45_V);
      break;
    }
    std::normal_distribution dist{static_cast<float>(vdc_voltage.get()), 0.1f};
    const Voltage v_dist = Voltage(dist(gen));
    return sensors::formula::inv_isolated_voltage(v_dist);
  }
  case ain_pin::i_mag_l_24: {
    const float duty = (__pwm_control.duty42 - 0.5) * 2.0;
    return mock_current(Current(duty * 38 / 2),
                        adc_isr::CURRENT_MEAS_GAIN_RIGHT);
  }
  case ain_pin::i_mag_r_25: {
    const float duty = (__pwm_control.duty42 - 0.5) * 2.0;
    return mock_current(Current(duty * 38 / 2),
                        adc_isr::CURRENT_MEAS_GAIN_RIGHT);
  }
  case ain_pin::i_mag_total: {
    const float duty1 = (__pwm_control.duty13 - 0.5) * 2.0;
    const float duty2 = (__pwm_control.duty42 - 0.5) * 2.0;
    return mock_current(Current((duty1 + duty2) * 38 / 2),
                        sensors::input_current::INPUT_CURRENT_GAIN);
  }
  default:
    assert(false && "trying to read a invalid pin");
    return 0_V;
  }
}

static std::normal_distribution mcu_temperature{273.15 + 24.0, 1.0};

Temperature guidance_board::read_mcu_temperature() {
  return Temperature(mcu_temperature(gen));
}

bool FLASHMEM guidance_board::register_periodic_reading(
    const Time &period, ain_pin pin, void (*on_value)(const Voltage &v)) {
  return ain_scheduler.schedule(period, AinSchedulerJob{
                                            .pin = pin,
                                            .on_value = on_value,
                                        });
}

void FASTRUN guidance_board::set_digital(ctrl_pin pin, bool state) {
  // pass
}

void guidance_board::delay(Duration delta) {
  std::this_thread::sleep_for(
      std::chrono::duration<uint32_t, std::micro>(delta.as_us()));
}

guidance_board::InterruptLock FASTRUN guidance_board::InterruptLock::acquire() {
  return InterruptLock();
}

void FASTRUN guidance_board::InterruptLock::release() { m_acquired = false; }
FASTRUN guidance_board::InterruptLock::~InterruptLock() { m_acquired = false; }

extern void adc_etc_done0_isr(AdcTrigRes);
extern void adc_etc_done1_isr(AdcTrigRes);

using namespace std::chrono;
static high_resolution_clock::time_point last_sim =
    high_resolution_clock::now();

void guidance_board::update() {
  ain_scheduler.update_continue();

  if (isr_interval.next()) {
    AdcTrigRes res;
    if (pwm::trig0_is_enabled()) {
      adc_etc_done0_isr(res);
    }
    if (pwm::trig1_is_enabled()) {
    }
  }
}
