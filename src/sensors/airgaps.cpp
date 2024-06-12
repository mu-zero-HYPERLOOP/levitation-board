#include "sensors/airgaps.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "firmware/adc_etc.h"
#include "firmware/guidance_board.h"
#include "print.h"
#include "sensors/formula/displacement420.h"
#include "util/boxcar.h"
#include <cassert>


static DMAMEM BoxcarFilter<Distance, 100> left_filter(0_mm);
static DMAMEM BoxcarFilter<Distance, 100> right_filter(0_mm);

static void on_left_disp(const Voltage &v) {
  const Current i = v / sensors::airgaps::R_MEAS;
  const Distance disp = sensors::formula::displacement420(i) - 29_mm;
  left_filter.push(disp);
  canzero_set_airgap_left(left_filter.get() / 1_mm);
}

static void on_right_disp(const Voltage &v) {
  const Current i = v / sensors::airgaps::R_MEAS;
  const Distance disp = sensors::formula::displacement420(i) - 29_mm;
  right_filter.push(disp);
  canzero_set_airgap_right(right_filter.get() / 1_mm);
}

void sensors::airgaps::begin() {
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::disp_sense_mag_l_19, on_left_disp));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::disp_sense_mag_r_17, on_right_disp));
}

void sensors::airgaps::calibrate() {
  for (size_t i = 0; i < left_filter.size(); ++i){
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_l_19);
    on_left_disp(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < right_filter.size(); ++i){
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_r_17);
    on_right_disp(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
}

void sensors::airgaps::update() {
}
