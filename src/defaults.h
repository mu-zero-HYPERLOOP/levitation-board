#pragma once

#include "canzero/canzero.h"

static void can_defaults() {
  canzero_set_error_arming_failed(error_flag_OK);
  canzero_set_error_heartbeat_miss(error_flag_OK);
  canzero_set_error_precharge_failed(error_flag_OK);

  canzero_set_error_level_mcu_temperature(error_level_OK);
  canzero_set_error_level_vdc_voltage(error_level_OK);
  canzero_set_error_level_input_current(error_level_OK);
  canzero_set_error_level_magnet_current_left(error_level_OK);
  canzero_set_error_level_magnet_current_right(error_level_OK);
  canzero_set_error_level_magnet_temperature_left(error_level_OK);
  canzero_set_error_level_magnet_temperature_right(error_level_OK);

  canzero_set_ignore_45v(bool_t_FALSE);
}
