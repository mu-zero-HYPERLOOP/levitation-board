#include "adc_isr.h"
#include "canzero/canzero.h"
#include "control.h"
#include "error_level_range_check.h"
#include "firmware/adc_etc.h"
#include "print.h"
#include "sensors/airgaps.h"
#include "sensors/formula/current_sense.h"
#include "sensors/formula/displacement420.h"
#include "sensors/formula/isolated_voltage.h"
#include "util/boxcar.h"
#include "util/interval.h"
#include <avr/pgmspace.h>

using namespace adc_isr;

static ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check_current_left(canzero_get_current_left,
                             canzero_get_error_level_config_magnet_current,
                             canzero_set_error_level_magnet_current_left);

static ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check_current_right(canzero_get_current_right,
                              canzero_get_error_level_config_magnet_current,
                              canzero_set_error_level_magnet_current_right);

static Interval interval(100_Hz);
static BoxcarFilter<Current, 1000> left_cali_offset(0_A);
static BoxcarFilter<Current, 1000> right_cali_offset(0_A);

void adc_isr::begin() {
  canzero_set_gamepad_x_down(bool_t_FALSE);
  // NOTE: if we decise to use any filters initalize them here!
  canzero_set_current_left(0);
  canzero_set_current_right(0);
  canzero_set_error_level_magnet_current_left(error_level_OK);
  canzero_set_error_level_magnet_current_right(error_level_OK);
  canzero_set_error_level_config_magnet_current(error_level_config{
      .m_info_thresh = 20,
      .m_info_timeout = 1,
      .m_warning_thresh = 30,
      .m_warning_timeout = 1,
      .m_error_thresh = 30,
      .m_error_timeout = 3,
      .m_ignore_info = bool_t_FALSE,
      .m_ignore_warning = bool_t_FALSE,
      .m_ignore_error = bool_t_FALSE,
  });
}

void adc_etc_done0_isr(AdcTrigRes res) {
  // software averages are cringe just use hardware averages!
  const Voltage v_i_mag_r = res.trig_res(TRIG0, 0);
  const Voltage v_i_mag_l = res.trig_res(TRIG0, 1);

  const Voltage v_disp_sense_mag_r = res.trig_res(TRIG4, 0);
  const Voltage v_disp_sense_mag_l = res.trig_res(TRIG4, 1);

  // Current sense
  const Current i_mag_r = sensors::formula::current_sense(
      v_i_mag_r, CURRENT_MEAS_GAIN_RIGHT, 1_mOhm) - right_cali_offset.get();
  const Current i_mag_l = sensors::formula::current_sense(
      v_i_mag_l, CURRENT_MEAS_GAIN_LEFT, 1_mOhm) - left_cali_offset.get();

  const Distance disp_sense_mag_l =
      sensors::airgaps::conv_left(v_disp_sense_mag_l);

  const Distance disp_sense_mag_r =
      sensors::airgaps::conv_right(v_disp_sense_mag_r);

  const GuidancePwmControl pwmControl = control::control_loop(
      i_mag_l, i_mag_r, disp_sense_mag_l, disp_sense_mag_r);

  pwm::control(static_cast<PwmControl>(pwmControl));
}

void adc_isr::update() {
  error_check_current_left.check();
  error_check_current_right.check();

  if (canzero_get_state() == levitation_state_READY && interval.next()) {
    Current cleft(canzero_get_current_left());
    Current cright(canzero_get_current_right());
    left_cali_offset.push(cleft);
    right_cali_offset.push(cright);
  } else {
    interval.reset();
  }
}
