#pragma once
#include <cinttypes>

enum class ctrl_pin : uint8_t {
  precharge_start_32 = 32,
  precharge_done_31 = 31,
  sdc_trig_37 = 37,
};

enum class ain_pin : uint8_t{
  // modified
  disp_sense_mag_l_19 = 19,
  //disp_sense_mag_l_19 = 18,  This is the actual on the PCB design
  disp_sense_lim_l_18 = 17,

  // modified, as this way the cabling is cleaner
  disp_sense_mag_r_17 = 18,
  //disp_sense_mag_r_17 = 19,  This is the actual on the PCB design
  disp_sense_lim_r_16 = 16,

  temp_sense_l2_21 = 21,
  temp_sense_l1_20 = 20,
  temp_sense_r2_15 = 15,
  temp_sense_r1_14 = 14,

  vdc_sense_40 = 40,

  i_mag_l_24 = 24,
  i_mag_r_25 = 25,
  i_mag_total = 26,
};

enum pwm_pin : uint8_t{
  left_high_l = 2,
  left_low_l = 3,
  left_high_r = 6,
  left_low_r = 9,

  right_high_l = 8,
  right_low_l = 7,
  right_high_r = 33,
  right_low_r = 4,
};


//////////////////////////// PWM Pins ////////////////////////////////

///// LEFT MAGNET //////

// FlexPWM4 Module2
// LEFT_HIN_L: Pin2 HIGH
// LEFT_LIN_L: Pin3 LOW

// FlexPWM2 Module2
// LEFT_HIN_R: Pin6 HIGH
// LEFT_LIN_R: Pin9 LOW

///// RIGHT MAGNET //////

// FlexPWM1 Module3
// RIGHT_HIN_L: Pin8 HIGH
// RIGHT_LIN_L: Pin7 LOW

// New in season five
// FIXME might not be the correct high / low pin assignment
// FlexPWM2 Module0
// RIGHT_HIN_R: Pin4 HIGH
// RIGHT_LIN_R: Pin33 LOW