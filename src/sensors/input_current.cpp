#include "sensors/input_current.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "error_level_range_check.h"
#include "firmware/guidance_board.h"
#include "sensors/formula/current_sense.h"
#include "util/boxcar.h"
#include "util/metrics.h"
#include <cassert>

static DMAMEM ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check(canzero_get_input_current,
                canzero_get_error_level_config_input_current,
                canzero_set_error_level_input_current);

static DMAMEM BoxcarFilter<Current, 1000> filter(0_A);

static void on_value(const Voltage &v) {
  Current i;
  if (canzero_get_state() == levitation_state_STOP ||
      canzero_get_state() == levitation_state_START ||
      canzero_get_state() == levitation_state_CONTROL) {
    /*i = sensors::formula::current_sense(*/
    /*    v, sensors::input_current::INPUT_CURRENT_GAIN, 1_mOhm);*/
    i  = 0_A;
    const bool sensible = i <= 100_A && i >= -20_A;
    canzero_set_error_input_current_invalid(sensible ? error_flag_OK
                                                     : error_flag_ERROR);
  } else {
    i = 0_A;
    filter.reset(0_A);
  }
  filter.push(i);
  canzero_set_input_current(static_cast<float>(filter.get()));
}

void sensors::input_current::begin() {
  canzero_set_input_current(0);
  canzero_set_error_level_input_current(error_level_OK);
  canzero_set_error_input_current_invalid(error_flag_OK);
  canzero_set_error_level_config_input_current(error_level_config{
      .m_info_thresh = 20,
      .m_info_timeout = 2,
      .m_warning_thresh = 40,
      .m_warning_timeout = 1,
      .m_error_thresh = 45,
      .m_error_timeout = 1,
      .m_ignore_info = bool_t_FALSE,
      .m_ignore_warning = bool_t_FALSE,
      .m_ignore_error = bool_t_FALSE,
  });
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::i_mag_total, on_value));
}

void sensors::input_current::calibrate() {
  for (size_t i = 0; i < filter.size(); ++i) {
    on_value(guidance_board::sync_read(ain_pin::i_mag_total));
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
}

void sensors::input_current::update() { error_check.check(); }
