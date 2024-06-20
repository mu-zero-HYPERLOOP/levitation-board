
#include "canzero/canzero.h"
#include "print.h"
#include <cinttypes>


void canzero_can0_wdg_timeout(uint8_t node_id) {
  debugPrintf("CAN0 Heartbeat Miss %u\n", node_id);
  canzero_set_error_heartbeat_miss(error_flag_ERROR);
}

void canzero_can1_wdg_timeout(uint8_t node_id) {
  debugPrintf("CAN1 Heartbeat Miss %u\n", node_id);
  canzero_set_error_heartbeat_miss(error_flag_ERROR);
}
