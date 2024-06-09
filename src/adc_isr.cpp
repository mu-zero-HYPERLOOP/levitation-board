#include "adc_isr.h"
#include "canzero/canzero.h"
#include "control.h"
#include "error_level_range_check.h"
#include "firmware/adc_etc.h"
#include "print.h"
#include "sensors/formula/current_sense.h"
#include "sensors/formula/displacement420.h"
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

void adc_isr::begin() {
  canzero_set_gamepad_x_down(bool_t_FALSE);
  // NOTE: if we decise to use any filters initalize them here!
  canzero_set_current_left(0);
  canzero_set_current_right(0);
  canzero_set_error_level_magnet_current_left(error_level_OK);
  canzero_set_error_level_magnet_current_right(error_level_OK);
  canzero_set_error_level_config_magnet_current(error_level_config{
      .m_info_thresh = 15,
      .m_info_timeout = 2,
      .m_warning_thresh = 20,
      .m_warning_timeout = 1,
      .m_error_thresh = 25,
      .m_error_timeout = 1,
      .m_ignore_info = bool_t_FALSE,
      .m_ignore_warning = bool_t_FALSE,
      .m_ignore_error = bool_t_FALSE,
  });
}

void adc_etc_done0_isr(AdcTrigRes res) {

  const Voltage v_disp_sense_mag_r_1 = res.trig_res(TRIG0, 0);
  const Voltage v_i_mag_l_1 = res.trig_res(TRIG0, 1);
  const Voltage v_i_mag_l_2 = res.trig_res(TRIG0, 2);
  const Voltage v_i_mag_l_3 = res.trig_res(TRIG0, 3);
  const Voltage v_i_mag_l_4 = res.trig_res(TRIG0, 4);
  const Voltage v_disp_sense_mag_r_2 = res.trig_res(TRIG0, 5);
  const Voltage v_i_mag_l = (v_i_mag_l_1 + v_i_mag_l_2 + v_i_mag_l_3 +
                             v_i_mag_l_4) /
                            4.0f;
  const Voltage v_disp_sense_mag_r = (v_disp_sense_mag_r_1 + v_disp_sense_mag_r_2) / 2.0f;

  const Voltage v_disp_sense_mag_l_1 = res.trig_res(TRIG4, 0); 
  const Voltage v_i_mag_r_1 = res.trig_res(TRIG4, 1);
  const Voltage v_i_mag_r_2 = res.trig_res(TRIG4, 2);
  const Voltage v_i_mag_r_3 = res.trig_res(TRIG4, 3);
  const Voltage v_i_mag_r_4 = res.trig_res(TRIG4, 4);
  const Voltage v_disp_sense_mag_l_2 = res.trig_res(TRIG4, 5);
  const Voltage v_i_mag_r = (v_i_mag_r_1 + v_i_mag_r_2 + v_i_mag_r_3 +
                             v_i_mag_r_4) /
                            4.0f;
  const Voltage v_disp_sense_mag_l = (v_disp_sense_mag_l_1 + v_disp_sense_mag_l_2) / 2.0f;

  // Current sense
  const Current i_mag_r =
      sensors::formula::current_sense(v_i_mag_r, CURRENT_MEAS_GAIN_RIGHT);
  const Current i_mag_l =
      sensors::formula::current_sense(v_i_mag_l, CURRENT_MEAS_GAIN_LEFT);

  const Current i_disp_sense_mag_l = v_disp_sense_mag_l / DISP_MEAS_R;
  const Distance disp_sense_mag_l =
      sensors::formula::displacement420(i_disp_sense_mag_l);

  const Current i_disp_sense_mag_r = v_disp_sense_mag_r / DISP_MEAS_R;
  const Distance disp_sense_mag_r =
      sensors::formula::displacement420(i_disp_sense_mag_r);

  const GuidancePwmControl pwmControl = control::control_loop(
      i_mag_l, i_mag_r, disp_sense_mag_l, disp_sense_mag_r);

  pwm::control(static_cast<PwmControl>(pwmControl));
}

void adc_isr::update() {
  error_check_current_left.check();
  error_check_current_right.check();
}
