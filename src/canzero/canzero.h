#ifndef CANZERO_H
#define CANZERO_H
#include "inttypes.h"
#include "stddef.h"
typedef struct {
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
} get_req_header;
typedef struct {
  uint8_t sof;
  uint8_t eof;
  uint8_t toggle;
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
} set_req_header;
typedef enum {
  mlu_command_NONE = 0,
  mlu_command_PRECHARGE = 1,
  mlu_command_START = 2,
  mlu_command_STOP = 3,
  mlu_command_ABORT = 4,
  mlu_command_DISCONNECT = 5,
} mlu_command;
typedef enum {
  node_id_mlu1 = 0,
  node_id_master = 1,
  node_id_mlu2 = 2,
  node_id_mlu3 = 3,
  node_id_mlu4 = 4,
  node_id_mlu5 = 5,
  node_id_mlu6 = 6,
  node_id_mgu1 = 7,
  node_id_mgu2 = 8,
  node_id_motor_driver = 9,
  node_id_pdu24 = 10,
  node_id_pdu12 = 11,
  node_id_input_board = 12,
  node_id_gamepad = 13,
} node_id;
typedef struct {
  uint8_t sof;
  uint8_t eof;
  uint8_t toggle;
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
} get_resp_header;
typedef enum {
  set_resp_erno_Success = 0,
  set_resp_erno_Error = 1,
} set_resp_erno;
typedef struct {
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
  set_resp_erno erno;
} set_resp_header;
typedef enum {
  mlu_state_INIT = 0,
  mlu_state_IDLE = 1,
  mlu_state_PRECHARGE = 2,
  mlu_state_READY = 3,
  mlu_state_START = 4,
  mlu_state_CONTROL = 5,
  mlu_state_STOP = 6,
} mlu_state;
typedef enum {
  sdc_status_OPEN = 0,
  sdc_status_CLOSED = 1,
} sdc_status;
typedef enum {
  error_flag_OK = 0,
  error_flag_ERROR = 1,
} error_flag;
typedef struct {
  float p_value;
  float i_value;
  float d_value;
} mlu_pid_values;
typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} canzero_frame;
typedef enum : uint32_t {
  CANZERO_FRAME_IDE_BIT = 0x40000000, // 1 << 30
  CANZERO_FRAME_RTR_BIT = 0x80000000, // 1 << 31
} can_frame_id_bits;
typedef struct {
  uint32_t mask;
  uint32_t id;
} canzero_can_filter;
extern void canzero_can0_setup(uint32_t baudrate, canzero_can_filter* filters, int filter_count);
extern void canzero_can0_send(canzero_frame* frame);
extern int canzero_can0_recv(canzero_frame* frame);
extern void canzero_can1_setup(uint32_t baudrate, canzero_can_filter* filters, int filter_count);
extern void canzero_can1_send(canzero_frame* frame);
extern int canzero_can1_recv(canzero_frame* frame);
extern void canzero_request_update(uint32_t time);
extern uint32_t canzero_get_time();
extern void canzero_enter_critical();
extern void canzero_exit_critical();
static inline mlu_state canzero_get_state() {
  extern mlu_state __oe_state;
  return __oe_state;
}
static inline mlu_command canzero_get_command() {
  extern mlu_command __oe_command;
  return __oe_command;
}
static inline mlu_pid_values canzero_get_control_config() {
  extern mlu_pid_values __oe_control_config;
  return __oe_control_config;
}
static inline sdc_status canzero_get_sdc_status() {
  extern sdc_status __oe_sdc_status;
  return __oe_sdc_status;
}
static inline float canzero_get_air_gap() {
  extern float __oe_air_gap;
  return __oe_air_gap;
}
static inline float canzero_get_target_force() {
  extern float __oe_target_force;
  return __oe_target_force;
}
static inline float canzero_get_dc_current() {
  extern float __oe_dc_current;
  return __oe_dc_current;
}
static inline float canzero_get_power_estimation() {
  extern float __oe_power_estimation;
  return __oe_power_estimation;
}
static inline float canzero_get_magnet_temperature() {
  extern float __oe_magnet_temperature;
  return __oe_magnet_temperature;
}
static inline float canzero_get_mcu_temperature() {
  extern float __oe_mcu_temperature;
  return __oe_mcu_temperature;
}
static inline float canzero_get_mosfet_temperature() {
  extern float __oe_mosfet_temperature;
  return __oe_mosfet_temperature;
}
static inline error_flag canzero_get_error_45V_over_voltage() {
  extern error_flag __oe_error_45V_over_voltage;
  return __oe_error_45V_over_voltage;
}
static inline error_flag canzero_get_error_45V_under_voltage() {
  extern error_flag __oe_error_45V_under_voltage;
  return __oe_error_45V_under_voltage;
}
static inline error_flag canzero_get_error_magnet_over_temperature() {
  extern error_flag __oe_error_magnet_over_temperature;
  return __oe_error_magnet_over_temperature;
}
static inline error_flag canzero_get_error_mcu_over_temperature() {
  extern error_flag __oe_error_mcu_over_temperature;
  return __oe_error_mcu_over_temperature;
}
static inline error_flag canzero_get_error_mosfet_over_temperature() {
  extern error_flag __oe_error_mosfet_over_temperature;
  return __oe_error_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_control_failure() {
  extern error_flag __oe_error_control_failure;
  return __oe_error_control_failure;
}
static inline error_flag canzero_get_warn_magnet_over_temperature() {
  extern error_flag __oe_warn_magnet_over_temperature;
  return __oe_warn_magnet_over_temperature;
}
static inline error_flag canzero_get_warn_mcu_over_temperature() {
  extern error_flag __oe_warn_mcu_over_temperature;
  return __oe_warn_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_mosfet_over_temperature() {
  extern error_flag __oe_warn_mosfet_over_temperature;
  return __oe_warn_mosfet_over_temperature;
}
static inline uint8_t canzero_get_mofset_temperature() {
  extern uint8_t __oe_mofset_temperature;
  return __oe_mofset_temperature;
}
typedef struct {
  get_resp_header header;
  uint32_t data;
} canzero_message_get_resp;
static const uint32_t canzero_message_get_resp_id = 0x13F;
typedef struct {
  set_resp_header header;
} canzero_message_set_resp;
static const uint32_t canzero_message_set_resp_id = 0x15F;
typedef struct {
  mlu_state state;
  mlu_command command;
} canzero_message_mlu1_stream_state;
static const uint32_t canzero_message_mlu1_stream_state_id = 0x78;
typedef struct {
  float air_gap;
  float target_force;
} canzero_message_mlu1_stream_control_info;
static const uint32_t canzero_message_mlu1_stream_control_info_id = 0x7E;
typedef struct {
  float dc_current;
  float power_estimation;
} canzero_message_mlu1_stream_power_estimation;
static const uint32_t canzero_message_mlu1_stream_power_estimation_id = 0x119;
typedef struct {
  float magnet_temperature;
  float mcu_temperature;
  uint8_t mofset_temperature;
} canzero_message_mlu1_stream_temperatures;
static const uint32_t canzero_message_mlu1_stream_temperatures_id = 0x9E;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mlu1_stream_sdc_status;
static const uint32_t canzero_message_mlu1_stream_sdc_status_id = 0x58;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_magnet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag error_mosfet_over_temperature;
  error_flag error_control_failure;
  error_flag warn_magnet_over_temperature;
  error_flag warn_mcu_over_temperature;
  error_flag warn_mosfet_over_temperature;
} canzero_message_mlu1_stream_errors;
static const uint32_t canzero_message_mlu1_stream_errors_id = 0xF9;
typedef struct {
  node_id node_id;
} canzero_message_heartbeat;
static const uint32_t canzero_message_heartbeat_id = 0x17B;
typedef struct {
  get_req_header header;
} canzero_message_get_req;
static const uint32_t canzero_message_get_req_id = 0x13B;
typedef struct {
  set_req_header header;
  uint32_t data;
} canzero_message_set_req;
static const uint32_t canzero_message_set_req_id = 0x15B;
typedef struct {
  mlu_command mlu_command;
} canzero_message_master_stream_mlu_control;
static const uint32_t canzero_message_master_stream_mlu_control_id = 0x52;
static void canzero_serialize_canzero_message_get_resp(canzero_message_get_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x13F;
  frame->dlc = 8;
  ((uint32_t*)data)[0] = (uint8_t)(msg->header.sof & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.eof & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.toggle & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13))) << 3;
  ((uint32_t*)data)[0] |= msg->header.client_id << 16;
  ((uint32_t*)data)[0] |= msg->header.server_id << 24;
  ((uint32_t*)data)[1] = msg->data;
}
static void canzero_serialize_canzero_message_set_resp(canzero_message_set_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x15F;
  frame->dlc = 4;
  ((uint32_t*)data)[0] = (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13)));
  ((uint32_t*)data)[0] |= msg->header.client_id << 13;
  ((uint32_t*)data)[0] |= msg->header.server_id << 21;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.erno & (0xFF >> (8 - 1))) << 29;
}
static void canzero_serialize_canzero_message_mlu1_stream_state(canzero_message_mlu1_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x78;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 3))) << 3;
}
static void canzero_serialize_canzero_message_mlu1_stream_control_info(canzero_message_mlu1_stream_control_info* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x7E;
  frame->dlc = 6;
  float air_gap_0 = (msg->air_gap - 0) / 0.000000004656612874161595;
  if (air_gap_0 > 4294967296.000) {
    air_gap_0 = 4294967296.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) air_gap_0;
  float target_force_32 = (msg->target_force - 0) / 0.0015259021896696422;
  if (target_force_32 > 4294901760.000) {
    target_force_32 = 4294901760.000;
  }
  ((uint32_t*)data)[1] = (uint32_t) target_force_32;
}
static void canzero_serialize_canzero_message_mlu1_stream_power_estimation(canzero_message_mlu1_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x119;
  frame->dlc = 4;
  float dc_current_0 = (msg->dc_current - -10) / 0.0009155413138017853;
  if (dc_current_0 > 4294901760.000) {
    dc_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_mlu1_stream_temperatures(canzero_message_mlu1_stream_temperatures* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x9E;
  frame->dlc = 3;
  float magnet_temperature_0 = (msg->magnet_temperature - -1) / 0.592156862745098;
  if (magnet_temperature_0 > 4278190080.000) {
    magnet_temperature_0 = 4278190080.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) magnet_temperature_0;
  float mcu_temperature_8 = (msg->mcu_temperature - -1) / 0.592156862745098;
  if (mcu_temperature_8 > 4278190080.000) {
    mcu_temperature_8 = 4278190080.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) mcu_temperature_8 << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->mofset_temperature & (0xFF >> (8 - 1))) << 16;
}
static void canzero_serialize_canzero_message_mlu1_stream_sdc_status(canzero_message_mlu1_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x58;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mlu1_stream_errors(canzero_message_mlu1_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xF9;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_failure & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 8;
}
static void canzero_serialize_canzero_message_heartbeat(canzero_message_heartbeat* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x17B;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->node_id & (0xFF >> (8 - 4)));
}
static void canzero_serialize_canzero_message_get_req(canzero_message_get_req* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x13B;
  frame->dlc = 4;
  ((uint32_t*)data)[0] = (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13)));
  ((uint32_t*)data)[0] |= msg->header.client_id << 13;
  ((uint32_t*)data)[0] |= msg->header.server_id << 21;
}
static void canzero_serialize_canzero_message_set_req(canzero_message_set_req* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x15B;
  frame->dlc = 8;
  ((uint32_t*)data)[0] = (uint8_t)(msg->header.sof & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.eof & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.toggle & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13))) << 3;
  ((uint32_t*)data)[0] |= msg->header.client_id << 16;
  ((uint32_t*)data)[0] |= msg->header.server_id << 24;
  ((uint32_t*)data)[1] = msg->data;
}
static void canzero_serialize_canzero_message_master_stream_mlu_control(canzero_message_master_stream_mlu_control* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x52;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->mlu_command & (0xFF >> (8 - 3)));
}
static void canzero_deserialize_canzero_message_get_resp(canzero_frame* frame, canzero_message_get_resp* msg) {
  uint8_t* data = frame->data;
  msg->header.sof = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->header.eof = ((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.toggle = ((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.od_index = ((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8)));
  msg->data = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 32)));
}
static void canzero_deserialize_canzero_message_set_resp(canzero_frame* frame, canzero_message_set_resp* msg) {
  uint8_t* data = frame->data;
  msg->header.od_index = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 21) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.erno = (set_resp_erno)((((uint32_t*)data)[0] >> 29) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu1_stream_state(canzero_frame* frame, canzero_message_mlu1_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mlu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->command = (mlu_command)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mlu1_stream_control_info(canzero_frame* frame, canzero_message_mlu1_stream_control_info* msg) {
  uint8_t* data = frame->data;
  msg->air_gap = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 32))) * 0.000000004656612874161595 + 0;
  msg->target_force = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 16))) * 0.0015259021896696422 + 0;
}
static void canzero_deserialize_canzero_message_mlu1_stream_power_estimation(canzero_frame* frame, canzero_message_mlu1_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mlu1_stream_temperatures(canzero_frame* frame, canzero_message_mlu1_stream_temperatures* msg) {
  uint8_t* data = frame->data;
  msg->magnet_temperature = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1;
  msg->mcu_temperature = ((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1;
  msg->mofset_temperature = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu1_stream_sdc_status(canzero_frame* frame, canzero_message_mlu1_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu1_stream_errors(canzero_frame* frame, canzero_message_mlu1_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_failure = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_heartbeat(canzero_frame* frame, canzero_message_heartbeat* msg) {
  uint8_t* data = frame->data;
  msg->node_id = (node_id)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
}
static void canzero_deserialize_canzero_message_get_req(canzero_frame* frame, canzero_message_get_req* msg) {
  uint8_t* data = frame->data;
  msg->header.od_index = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 21) & (0xFFFFFFFF >> (32 - 8)));
}
static void canzero_deserialize_canzero_message_set_req(canzero_frame* frame, canzero_message_set_req* msg) {
  uint8_t* data = frame->data;
  msg->header.sof = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->header.eof = ((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.toggle = ((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.od_index = ((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8)));
  msg->data = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 32)));
}
static void canzero_deserialize_canzero_message_master_stream_mlu_control(canzero_frame* frame, canzero_message_master_stream_mlu_control* msg) {
  uint8_t* data = frame->data;
  msg->mlu_command = (mlu_command)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
}
void canzero_can0_poll();
void canzero_can1_poll();
uint32_t canzero_update_continue(uint32_t delta_time);
void canzero_init();
void canzero_set_state(mlu_state value);
void canzero_set_command(mlu_command value);
static inline void canzero_set_control_config(mlu_pid_values value){
  extern mlu_pid_values __oe_control_config;
  __oe_control_config = value;
}
void canzero_set_sdc_status(sdc_status value);
static inline void canzero_set_air_gap(float value){
  extern float __oe_air_gap;
  __oe_air_gap = value;
}
static inline void canzero_set_target_force(float value){
  extern float __oe_target_force;
  __oe_target_force = value;
}
static inline void canzero_set_dc_current(float value){
  extern float __oe_dc_current;
  __oe_dc_current = value;
}
static inline void canzero_set_power_estimation(float value){
  extern float __oe_power_estimation;
  __oe_power_estimation = value;
}
static inline void canzero_set_magnet_temperature(float value){
  extern float __oe_magnet_temperature;
  __oe_magnet_temperature = value;
}
static inline void canzero_set_mcu_temperature(float value){
  extern float __oe_mcu_temperature;
  __oe_mcu_temperature = value;
}
static inline void canzero_set_mosfet_temperature(float value){
  extern float __oe_mosfet_temperature;
  __oe_mosfet_temperature = value;
}
void canzero_set_error_45V_over_voltage(error_flag value);
void canzero_set_error_45V_under_voltage(error_flag value);
void canzero_set_error_magnet_over_temperature(error_flag value);
void canzero_set_error_mcu_over_temperature(error_flag value);
void canzero_set_error_mosfet_over_temperature(error_flag value);
void canzero_set_error_control_failure(error_flag value);
void canzero_set_warn_magnet_over_temperature(error_flag value);
void canzero_set_warn_mcu_over_temperature(error_flag value);
void canzero_set_warn_mosfet_over_temperature(error_flag value);
static inline void canzero_set_mofset_temperature(uint8_t value){
  extern uint8_t __oe_mofset_temperature;
  __oe_mofset_temperature = value;
}
#endif