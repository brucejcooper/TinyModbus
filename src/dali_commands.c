#include "dali.h"

static void scan_result_received(dali_response_type_t err, uint16_t cmd, uint8_t response) {
    uint8_t addr = (cmd >> 9) & 0x3F;
    if (addr < 64) {
        switch (err) {
            case  DALI_RESPONSE_OK:
                dali_holding_regs.power[addr] = response;
                break;
            default:
                dali_holding_regs.power[addr] = err;
        }
        if (addr < 63) {
            dali_start_scan(addr+1);
        }
    }
}



void dali_start_scan(uint8_t addr) {
    dali_queue_command(addr << 9 | 0x1a0 , scan_result_received);
}
