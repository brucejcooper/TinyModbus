#ifndef __RS485_H__
#define __RS485_H__

#include <stdint.h>
#include <stdbool.h>

extern void rs485_init(uint8_t deviceId);
extern volatile bool modbus_idle;
extern void rs485Timer_timeout();
extern void rs485_process_command();
#endif