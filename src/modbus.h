#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>
#include <stdbool.h>

extern volatile bool modbus_idle;

void modbus_init(uint8_t slave_addr);

// Called from main loop — processes one complete Modbus frame if available.
void modbus_process(void);

// Called from TCA0_OVF ISR — drives the inter-character silence timer.
void modbus_tick(void);

#endif
