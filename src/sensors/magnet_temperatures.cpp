#include "sensors/magnet_temperatures.h"
#include "avr/pgmspace.h"
#include "print.h"
#include "canzero/canzero.h"
#include "error_level_range_check.h"
#include "firmware/guidance_board.h"
#include "sensors/formula/ptx.h"
#include "sensors/formula/voltage_divider.h"
#include "util/boxcar.h"
#include "util/metrics.h"
#include <algorithm>
#include <cassert>

static DMAMEM BoxcarFilter<Temperature, 1000> left_filter(24_Celcius);
static DMAMEM BoxcarFilter<Temperature, 1000> right_filter(24_Celcius);

static DMAMEM ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check_left(canzero_get_magnet_temperature_left,
                     canzero_get_error_level_config_magnet_temperature,
                     canzero_set_error_level_magnet_temperature_left);

static DMAMEM ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check_right(canzero_get_magnet_temperature_right,
                      canzero_get_error_level_config_magnet_temperature,
                      canzero_set_error_level_magnet_temperature_right);

static void on_left_value(const Voltage &v) {
  if (v < 0.1_V){
    canzero_set_error_magnet_temperature_left_invalid(error_flag_ERROR);
    canzero_set_magnet_temperature_left(0);
    return;
  }
  canzero_set_error_magnet_temperature_left_invalid(error_flag_OK);
  const Resistance r_ptx = sensors::formula::r1_of_voltage_divider(
      4.9_V, v, sensors::magnet_temperatures::R_MEAS);
  const Temperature temp =
      sensors::formula::ptx(r_ptx, sensors::magnet_temperatures::PTX_R0_L) - 15_K;
  left_filter.push(temp);
  canzero_set_magnet_temperature_left(
      static_cast<float>(left_filter.get() - 0_Celcius));
}

static void on_right_value(const Voltage &v) {
  if (v < 0.1_V){
    canzero_set_error_magnet_temperature_right_invalid(error_flag_ERROR);
    canzero_set_magnet_temperature_right(0);
    return;
  }
  canzero_set_error_magnet_temperature_right_invalid(error_flag_OK);
  const Resistance r_ptx = sensors::formula::r1_of_voltage_divider(
      4.9_V, v, sensors::magnet_temperatures::R_MEAS);
  const Temperature temp =
      sensors::formula::ptx(r_ptx, sensors::magnet_temperatures::PTX_R0_R) - 15_K;
  right_filter.push(temp);
  canzero_set_magnet_temperature_right(
      static_cast<float>(right_filter.get() - 0_Celcius));
}

void sensors::magnet_temperatures::begin() {
  canzero_set_magnet_temperature_left(0);
  canzero_set_magnet_temperature_right(0);
  canzero_set_error_level_magnet_temperature_left(error_level_OK);
  canzero_set_error_level_magnet_temperature_right(error_level_OK);
  canzero_set_error_level_config_magnet_temperature(error_level_config{
      .m_info_thresh = 60,
      .m_info_timeout = 5,
      .m_warning_thresh = 80,
      .m_warning_timeout = 5,
      .m_error_thresh = 100,
      .m_error_timeout = 5,
      .m_ignore_info = bool_t_FALSE,
      .m_ignore_warning = bool_t_FALSE,
      .m_ignore_error = bool_t_FALSE,
  });

  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::temp_sense_l1_20, on_left_value));

  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::temp_sense_r1_14, on_right_value));
}

void sensors::magnet_temperatures::calibrate() {
  for (size_t i = 0; i < left_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::temp_sense_l1_20);
    on_left_value(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < right_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::temp_sense_r1_14);
    on_right_value(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
}

void sensors::magnet_temperatures::update() {
  error_check_left.check();
  error_check_right.check();
}
