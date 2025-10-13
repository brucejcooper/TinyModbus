#include "dali.h"

dali_all_holding_registers_t dali_holding_regs;
uint16_t dali_input_regs[DALI_MAX_DEVICES];



void dali_state_init() {
    for (int i = 0; i < DALI_MAX_DEVICES; i++) {
        dali_holding_regs.power[i] = 0xFFFF;
    }
}