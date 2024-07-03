#include "sensors/airgaps.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "sensors/formula/displacement420.h"
#include "util/boxcar.h"
#include "util/dist_estimation.h"
#include <cassert>

static DMAMEM DistEstimation<Distance, 1000> left_var(0_mm);
static DMAMEM DistEstimation<Distance, 1000> right_var(0_mm);
static DMAMEM BoxcarFilter<Distance, 1> left_filter(0_mm);
static DMAMEM BoxcarFilter<Distance, 1> right_filter(0_mm);

static Distance offset_left = 0_m;
static Distance offset_right = 0_m;


Distance sensors::airgaps::conv_left(Voltage v){

  const Current i = v / sensors::airgaps::R_MEAS;
  return sensors::formula::displacement420(i) + offset_left;
}
Distance sensors::airgaps::conv_right(Voltage v){
  const Current i = v / sensors::airgaps::R_MEAS;
  return sensors::formula::displacement420(i) + offset_right;
}


static void on_left_disp(const Voltage &v) {
  const Distance disp = sensors::airgaps::conv_left(v);
  left_filter.push(disp);
  left_var.push(disp);
  canzero_set_airgap_left(disp / 1_mm);
  canzero_set_airgap_left_variance(left_var.variance() * 1e3);
}

static void on_right_disp(const Voltage &v) {
  const Distance disp = sensors::airgaps::conv_right(v);
  right_filter.push(disp);
  right_var.push(disp);
  canzero_set_airgap_right(disp / 1_mm);
  canzero_set_airgap_right_variance(left_var.variance() * 1e3);
}

void sensors::airgaps::begin() {
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::disp_sense_mag_l_19, on_left_disp));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::disp_sense_mag_r_17, on_right_disp));
}

void sensors::airgaps::calibrate() {
  offset_left = 0_m;
  offset_right = 0_m;
  /* BoxcarFilter<Distance, 1000> cali_left_filter(0_mm); */
  /* for (size_t k = 0; k < cali_left_filter.size(); ++k) { */
  /*   const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_l_19); */
  /* const Current i = v / sensors::airgaps::R_MEAS; */
  /*   const Distance disp = sensors::formula::displacement420(i); */
  /*   cali_left_filter.push(disp); */
  /*   guidance_board::delay(10_us); */
  /* } */
  /* Distance left_target = sensors::airgaps::ground_right(); */
  /* offset_left = left_target - cali_left_filter.get(); */
  /*  */
  /*  */
  /* BoxcarFilter<Distance, 1000> cali_right_filter(0_mm); */
  /* for (size_t k = 0; k < cali_right_filter.size(); ++k) { */
  /*   const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_r_17); */
  /* const Current i = v / sensors::airgaps::R_MEAS; */
  /*   const Distance disp = sensors::formula::displacement420(i); */
  /*   cali_right_filter.push(disp); */
  /*   guidance_board::delay(10_us); */
  /* } */
  /* Distance right_target = sensors::airgaps::ground_right(); */
  /* offset_right = right_target - cali_right_filter.get(); */

  offset_right = 0_mm;
  offset_left = 0_mm;

  for (size_t i = 0; i < left_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_l_19);
    on_left_disp(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < right_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_r_17);
    on_right_disp(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }


  canzero_set_airgap_left(left_filter.get() / 1_mm);
  canzero_set_airgap_right(right_filter.get() / 1_mm);

}


void sensors::airgaps::update() {}
