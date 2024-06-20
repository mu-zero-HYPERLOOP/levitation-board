#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "print.h"
#include "sdc_brake.h"


/**
 * This function get's invoked if a assertion fails
 */
void __assert_func(const char *filename, int line, const char *assert_func,
                   const char *expr) {
  sdc_brake::brake_immediatly();
  canzero_set_assertion_fault(error_flag_ERROR);
  canzero_update_continue(canzero_get_time());
  debugPrintf("ASSERTION FAILED %s:%i\n", filename, line);
  while (true) {
    guidance_board::delay(1_s);
  }
}
