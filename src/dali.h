/* 
  DALI master, useable as a multi-switch controller.  Programmed (via UPDI, or UART?) to respond to button presses
  by sending out DALI commands to dim 
*/

#ifndef __DALI_H__
#define __DALI_H__

#include <inttypes.h>
#include <stdbool.h>
#include <avr/io.h>

typedef enum { 
    DALI_OK = 0, 
    DALI_ERR_COLLISION_DETECTED,
    DALI_ERR_INVALID_STATE,
    DALI_ERR_NO_START_BIT,
    DALI_ERR_NOT_FOR_ME,
    DALI_ERR_CORRUPT_READ,
    DALI_ERR_QUEUEFULL,
} dali_result_t;


typedef enum {
    RESPONSE,
    NO_RESPONSE,
    COLLISION,
} dali_command_result_t;


#define DALI_BAUD                   (1200)
#define DALI_BIT_USECS              (1000000.0/DALI_BAUD)
#define DALI_HALF_BIT_USECS         (DALI_BIT_USECS/2.0)
#define DALI_MAX_DEVICES            (64)
#define DALI_DECODE_ADDR(cmd)       (((cmd) >> 9) & 0x3F)


// The DALI configuration - PB3 for transmit - PB2 for receive.
#define DALI_PORT PORTB
#define DALI_TX_bm PIN0_bm
#define DALI_RX_bm PIN1_bm
#define DALI_RX_PINCTRL PIN1CTRL

typedef enum {
  DALI_RESPONSE_OK = 0,
  DALI_RESPONSE_NAK = 404,
  DALI_RESPONSE_COLLISION = 409,
  DALI_RESPONSE_MALFORMED = 500,
  DALI_RESPONSE_PENDING = 255,
} dali_response_type_t;


typedef struct {
    uint8_t level;
    uint8_t status;
} dali_pwr_bank_t;

typedef struct {
    uint8_t min;
    uint8_t max;
} dali_minmax_bank_t;

typedef struct {
    uint8_t rate;
    uint8_t time;
} dali_fade_bank_t;

typedef struct {
    uint8_t powerOnLevel;
    uint8_t systemFailureLevel;
} dali_initialvalues_bank_t;


typedef union {
    uint16_t shortVal;
    struct {
        // ATMEL is little endian
        uint8_t low;
        uint8_t high;
    };
} modbus_register_t;

typedef struct {
    modbus_register_t power[DALI_MAX_DEVICES];
    // dali_minmax_bank_t minmax[DALI_MAX_DEVICES];
    // dali_fade_bank_t fade[DALI_MAX_DEVICES];
    // dali_initialvalues_bank_t initialValues[DALI_MAX_DEVICES];
    // uint16_t groups[DALI_MAX_DEVICES];
} dali_all_holding_registers_t;

struct dali_command_t;

typedef void (*dali_callback_t)(dali_response_type_t responseType, uint16_t cmd, uint8_t lastResponse);
typedef bool (*dali_callback_next_t)(struct dali_command_t *cmd, dali_response_type_t responseType, uint8_t response);


typedef struct dali_command_t {
    uint16_t cmd;
    dali_callback_t callback;
    uint8_t repeat;
} dali_command_t;



// extern uint16_t dali_input_regs[DALI_MAX_DEVICES];
// #define DALI_MAX_INPUT_REGISTER (sizeof(dali_input_regs) / sizeof(uint16_t))

extern dali_all_holding_registers_t dali_holding_regs;
#define DALI_MAX_HOLDING_REGISTER (sizeof(dali_holding_regs) / sizeof(uint16_t))

extern void pulse_led(uint8_t times);

extern void dali_init();
uint8_t dali_is_bus_shorted();


// State management - each of these functions lives in its own source file. 
void dali_idle_state_enter();
extern void dali_transmitting_state_enter(dali_command_t *cmd);
void dali_wait_for_idle_state_enter();
void dali_receiving_state_enter();
void dali_waiting_for_response_state_enter();

void dali_call_on_rising_once(void (*callback)());
void dali_call_on_low_once(void (*callback)());
void dali_cancel_edge_detect();


// Receiving state.
void dali_state_receiving_prepare();
void dali_state_receiving_enter();


void dali_wait_for_response_state_enter();
extern volatile bool dali_idle;
void process_command_response(dali_response_type_t err, uint8_t res);
void dali_state_init();


// Commmands
extern void dali_start_background_scan(uint8_t addr);
extern void dali_goto_last_active(uint8_t addr, dali_callback_t callback);
extern void dali_off(uint8_t addr, dali_callback_t callback);
extern void dali_toggle(uint8_t addr, dali_callback_t callback);
extern void dali_set_level(uint8_t addr, uint8_t level, dali_callback_t callback);
extern void dali_read_level_and_status(uint8_t addr, dali_callback_t callback);
extern void dali_custom_command(uint16_t cmd, dali_callback_t responseCallback, uint8_t repeat);


#endif
