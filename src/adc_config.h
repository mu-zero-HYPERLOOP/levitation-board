#pragma once

#include "firmware/adc_etc.h"

static void adc_config() {

  /*
   * BIG NOTE!!!!
   * Not all pins can be read from all trigger chains 
   * trigger chains 0-3 are wired to ADC1!
   * and all chains 4-7 are wired to ADC2!
   * YOU HAVE TO LOOK AT THE teensy4.1 datasheet to see, 
   * which pins are connected to which ADC!
  * see https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf section : 66.3
   */
  TrigChainInfo chains[2];
  chains[0].trig_num = TRIG0;
  ain_pin chain0_pins[] = {
      ain_pin::i_mag_r_25, ain_pin::i_mag_l_24,
  };
  chains[0].read_pins = chain0_pins;
  chains[0].chain_length = sizeof(chain0_pins) / sizeof(ain_pin);
  chains[0].chain_priority = 0;
  chains[0].software_trig = false;
  chains[0].trig_sync = true;
  chains[0].intr = DONE0;

  chains[1].trig_num = TRIG4;
  ain_pin chain4_pins[] = {
      ain_pin::disp_sense_mag_r_17, ain_pin::disp_sense_mag_l_19,
  };
  chains[1].read_pins = chain4_pins;
  chains[1].chain_length = sizeof(chain4_pins) / sizeof(ain_pin);
  chains[1].chain_priority = 0;
  chains[1].software_trig = false;
  chains[1].trig_sync = false;
  chains[1].intr = NONE;

  AdcEtcBeginInfo adcBeginInfo{};
  adcBeginInfo.adc1_avg = HwAvg::SAMPLE_8;
  adcBeginInfo.adc1_clock_div = AdcClockDivider::NO_DIV;
  adcBeginInfo.adc1_high_speed = true;
  adcBeginInfo.adc1_sample_time = PERIOD_25;
  adcBeginInfo.adc1_resolution = BIT_12;
  adcBeginInfo.adc2_avg = adcBeginInfo.adc1_avg;
  adcBeginInfo.adc2_clock_div = adcBeginInfo.adc1_clock_div;
  adcBeginInfo.adc2_high_speed = adcBeginInfo.adc1_high_speed;
  adcBeginInfo.adc2_sample_time = adcBeginInfo.adc1_sample_time;
  adcBeginInfo.adc2_resolution = adcBeginInfo.adc1_resolution;
  adcBeginInfo.chains = chains;
  adcBeginInfo.num_chains = sizeof(chains) / sizeof(TrigChainInfo);

  adc_etc::begin(adcBeginInfo);
}
