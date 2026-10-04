// frame_pair.h — is a CRC-bad capture really two good frames glued together?
//
// A slave that replies faster than our end-of-frame silence window leaves no
// gap we can see between the master's request and its reply, so we capture
// both as ONE frame and its CRC fails. On the house bus (2026-10-01) that is
// the Waveshare relay board at slave 1: every coil poll the controller sends
// it reached both gateways as `01 01 00 00 00 20 3d d2 | 01 01 04 ...` and was
// charged to crc_error — about one "error" per relay poll on a healthy bus.
//
// Nothing about such a capture is corrupt: it splits at some point into two
// frames that each pass their own CRC. Real line noise passing BOTH checks by
// chance is ~1 in 2^32. So a capture that splits this way is treated as two
// clean frames — and if neither is addressed to us, it is someone else's
// traffic and is ignored like any other. A capture that does NOT split is
// still a CRC error, always (the counting rule: a CRC-bad frame counts, since
// its address cannot be trusted).
//
// Pure C, no AVR dependency, so it runs in the host test (test/frame_pair_test.c).
// The CRC is passed in: the firmware uses modbus.c's PROGMEM-table crc16, so
// there is still one CRC implementation on the chip.
#ifndef FRAME_PAIR_H
#define FRAME_PAIR_H

#include <stdbool.h>
#include <stdint.h>

typedef uint16_t (*frame_crc_fn)(const uint8_t *data, uint8_t len);

// Smallest RTU frame: address + function code + CRC(2).
#define FRAME_MIN_LEN 4u

// True if buf[0..n) is exactly two CRC-valid frames back to back, each at
// least FRAME_MIN_LEN long. *split receives the length of the first.
bool frame_is_pair(const uint8_t *buf, uint8_t n, frame_crc_fn crc,
                   uint8_t *split);

#endif
