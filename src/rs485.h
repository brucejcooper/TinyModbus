#ifndef __RS485_H__
#define __RS485_H__

#include <stdint.h>
#include <stdbool.h>

extern void rs485_init(uint8_t deviceId);
extern bool rs485_process_command();

#endif