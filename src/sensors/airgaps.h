#pragma once

#include "canzero/canzero.h"
#include "util/metrics.h"
namespace sensors::airgaps {

constexpr Frequency MEAS_FREQUENCY = 1_kHz;

constexpr Resistance R_MEAS = 120_Ohm;

void begin();

void calibrate();

Distance conv_left(Voltage v);
Distance conv_right(Voltage v);

constexpr Distance ground_left() {
  if (CANZERO_NODE_ID == node_id_levitation_board1){
    return 16_mm;
  }else if (CANZERO_NODE_ID == node_id_levitation_board2){
    return 16_mm;
  }else if (CANZERO_NODE_ID == node_id_levitation_board3){
    return 16_mm;
  }
}

constexpr Distance ground_right(){
  if (CANZERO_NODE_ID == node_id_levitation_board1){
    return 16_mm;
  }else if (CANZERO_NODE_ID == node_id_levitation_board2){
    return 16_mm;
  }else if (CANZERO_NODE_ID == node_id_levitation_board3){
    return 16_mm;
  }
}

void update();

}
