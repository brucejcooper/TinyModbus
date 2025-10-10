#ifndef __relays_H__
#define __relays_H__

extern void relay_init();
extern void latchOutputs();
extern void setCoil(uint8_t addr, uint8_t val);
extern uint8_t getCoils(uint8_t addr);


#endif