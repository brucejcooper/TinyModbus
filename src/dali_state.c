#include "dali.h"

dali_all_holding_registers_t dali_holding_regs;



void dali_state_init() {
    for (int bank = 0; bank < DALI_NUM_REG_BANKS; bank++) {
        for (int addr = 0; addr < DALI_MAX_DEVICES; addr++) {
            dali_holding_regs.banks[bank][addr].shortVal = bank == 0 ? 0x00FF: 0x0000;
        }
    }
}