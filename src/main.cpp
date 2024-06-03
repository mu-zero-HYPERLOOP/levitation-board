#include "adc_config.h"
#include "adc_isr.h"
#include "canzero/canzero.h"
#include "control.h"
#include "firmware/guidance_board.h"
#include "fsm/fsm.h"
#include "pwm_brake.h"
#include "pwm_config.h"
#include "sdc_brake.h"
#include "sensors/input_current.h"
#include "sensors/magnet_temperatures.h"
#include "sensors/mcu_temperature.h"
#include "sensors/vdc.h"
#include "xbar_config.h"

int main() {
    
  canzero_init();
  fsm::begin();
  guidance_board::begin();
  adc_config();
  pwm_config();
  xbar_config();

  // Setup sensors
  sensors::input_current::begin();
  sensors::mcu_temperature::begin();
  sensors::magnet_temperatures::begin();
  sensors::vdc::begin();

  // Calibrate sensors
  sensors::input_current::calibrate();
  sensors::mcu_temperature::calibrate();
  sensors::magnet_temperatures::calibrate();
  sensors::vdc::calibrate();

  // Setup brakes
  pwm_brake::begin();
  sdc_brake::begin();

  // Setup control
  adc_isr::begin();
  control::begin();

  // init -> idle.
  fsm::finish_init(); 
  while(true){
    // Receive from CAN
    canzero_can0_poll();
    canzero_can1_poll();

    // Update firmware
    guidance_board::update();

    // Update sensors
    sensors::input_current::update();
    sensors::mcu_temperature::update();
    sensors::magnet_temperatures::update();
    sensors::vdc::update();

    // Update brakes
    sdc_brake::update();
    pwm_brake::update();

    // Update control
    adc_isr::update();
    control::update();

    // Update state machine
    fsm::update();

    // Send on CAN.
    canzero_update_continue(canzero_get_time());
  }


}
