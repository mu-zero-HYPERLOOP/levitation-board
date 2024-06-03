#include "fsm/error_handling.h"
#include "canzero/canzero.h"
#include <algorithm>
#include <array>

levitation_command fsm::error_handling::approve(levitation_command cmd) {



  const auto error_flags = std::array<error_flag, 2>{
    canzero_get_error_arming_failed(),
    canzero_get_error_precharge_failed(),
  };

  const auto max_error_flag_it = std::max_element(error_flags.begin(), error_flags.end());
  const error_flag max_error_flag = *max_error_flag_it;
  if (max_error_flag == error_flag_ERROR){
    // early bail out.
    return levitation_command_DISARM45;
  }

  const auto error_levels = std::array<error_level, 3>{
      canzero_get_error_level_mcu_temperature(),
      canzero_get_error_level_magnet_temperature_left(),
      canzero_get_error_level_magnet_temperature_right(),
  };
  const auto max_error_level_it = std::max_element(error_levels.begin(), error_levels.end());
  const error_level max_error_level = *max_error_level_it;

  const error_level error_level = std::max(max_error_level,
      (max_error_flag == error_flag_OK) ? error_level_OK : error_level_ERROR);
  switch (error_level){
  case error_level_OK:
  case error_level_INFO:
    return cmd;
  case error_level_WARNING:
    return levitation_command_ABORT;
  case error_level_ERROR:
    return levitation_command_DISARM45;
  default:
    return levitation_command_ABORT;
  }
}
