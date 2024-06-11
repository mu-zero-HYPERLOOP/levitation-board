#include "sensors/magnet_current.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "sensors/formula/current_sense.h"
#include "util/boxcar.h"
#include <cassert>

static DMAMEM BoxcarFilter<Current, 100> left_filter(0_A);
static DMAMEM BoxcarFilter<Current, 100> right_filter(0_A);

static void on_left_current(const Voltage &v) {
  const Current i = sensors::formula::current_sense(v, sensors::magnet_current::SENSE_GAIN, 1_mOhm);
  left_filter.push(i);
  canzero_set_current_left(static_cast<float>(left_filter.get()));
}

static void on_right_current(const Voltage &v) {
  const Current i = sensors::formula::current_sense(v, sensors::magnet_current::SENSE_GAIN, 1_mOhm);
  right_filter.push(i);
  canzero_set_current_right(static_cast<float>(right_filter.get()));
}

void sensors::magnet_current::begin() {
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::i_mag_l_24, on_left_current));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::i_mag_r_25, on_right_current));
}

void sensors::magnet_current::calibrate() {
  for (size_t i = 0; i < left_filter.size(); ++i){
    const Voltage v = guidance_board::sync_read(ain_pin::i_mag_l_24);
    on_left_current(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < left_filter.size(); ++i){
    const Voltage v = guidance_board::sync_read(ain_pin::i_mag_r_25);
    on_right_current(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
}

void sensors::magnet_current::update() {
}
