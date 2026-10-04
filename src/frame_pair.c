// frame_pair.c — see frame_pair.h. Not a hot path: runs in the main loop, only
// for captures that already failed their CRC.
#pragma GCC optimize("Os")

#include "frame_pair.h"

bool frame_is_pair(const uint8_t *buf, uint8_t n, frame_crc_fn crc,
                   uint8_t *split) {
    if (n < 2u * FRAME_MIN_LEN) return false;
    for (uint8_t k = FRAME_MIN_LEN; (uint8_t)(n - k) >= FRAME_MIN_LEN; k++) {
        if (crc(buf, k) == 0u && crc(buf + k, (uint8_t)(n - k)) == 0u) {
            *split = k;
            return true;
        }
    }
    return false;
}
