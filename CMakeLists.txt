######################################################################
# @author      : karl (karlsasssie@gmail.com)
# @file        : CMakeLists
# @created     : Friday Apr 07, 2023 15:39:05 CEST
######################################################################

cmake_minimum_required(VERSION 3.5)
project(levitation-board CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} 
  mock/firmware/adc_etc.cpp
  mock/firmware/guidance_board.cpp
  mock/firmware/pwm.cpp
  mock/firmware/xbar.cpp
  mock/print.cpp
  mock/can.cpp
  src/canzero/canzero.cpp
  src/fsm/states/arming45.cpp
  src/fsm/states/control.cpp
  src/fsm/states/idle.cpp
  src/fsm/states/init.cpp
  src/fsm/states/precharge.cpp
  src/fsm/states/disarming45.cpp
  src/fsm/states/ready.cpp
  src/fsm/states/start.cpp
  src/fsm/states/stop.cpp
  src/fsm/error_handling.cpp
  src/fsm/fsm.cpp
  src/sensors/airgaps.cpp
  src/sensors/input_current.cpp
  src/sensors/magnet_current.cpp
  src/sensors/magnet_temperatures.cpp
  src/sensors/mcu_temperature.cpp
  src/sensors/vdc.cpp
  src/adc_isr.cpp
  src/airgap_transition.cpp
  src/assert_err.cpp
  src/control.cpp
  src/main.cpp
  src/sdc_brake.cpp
  src/handle_heartbeat.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE src/ mock/)

