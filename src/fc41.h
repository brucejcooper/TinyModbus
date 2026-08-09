#ifndef FC41_H
#define FC41_H

/*
 * fc41.h — the FC 0x41 request wire format, in one place.
 *
 * THE function code: one user-defined code carries all queue traffic. A request
 * optionally enqueues one DALI command and optionally acks (deletes) the head of
 * the response queue; the reply reports the fate of the enqueue plus the current
 * head. QUEUE-PROTOCOL.md §1 is normative and this header is its transcription —
 * if the two ever disagree, the spec wins and this is the bug.
 *
 * MODBUS Application Protocol V1.1b reserves 65-72 and 100-110 for user
 * definition, so 0x41 is conformant rather than an extension. Framing, CRC,
 * silence and exception replies are all ordinary RTU; only the payload shape is
 * custom.
 *
 * PEER DEFINITION: controller/src/mbq_client.h — separate repo, so these
 * constants cannot be shared and are mirrored by hand. THE TWO MUST AGREE BIT
 * FOR BIT. They are a matched set with no fallback path (§8), and drift here
 * would not fail loudly: it would present as every command meaning something
 * slightly different from what was asked for. Change one, change the other, in
 * the same task.
 *
 * Every packed byte has the same shape — control in the top 3 bits, sequence in
 * the low 5 — and sequences run 1..31 with 0 reserved for "none":
 *
 *     bit  7    5 4          0
 *        ┌───────┬─────────────┐
 *        │ ctl:3 │    seq:5    │
 *        └───────┴─────────────┘
 *
 * REQUEST, 5 / 7 / 8 bytes (CRC included):
 *
 *   ┌─────┬──────┬─────────────────┬──────┬──────┬──────┬────────┬────────┐
 *   │slave│ 0x41 │ [flags:3|ack:5] │ addr │ data │ dtr0 │ CRC lo │ CRC hi │
 *   └─────┴──────┴─────────────────┴──────┴──────┴──────┴────────┴────────┘
 *                                   └── enqueue payload, 7-8 B frames ──┘
 *
 * REPLY, 6 or 7 bytes. The accept byte is present IFF the request attempted an
 * enqueue — i.e. the shape follows the REQUEST, not the outcome, so an A_FULL
 * reply is still 7 bytes. The master always knows which to expect because it
 * sent the request.
 *
 *   7 B: slave, 0x41, [astatus:3|aseq:5], [hstatus:3|hseq:5], resp8, CRC, CRC
 *   6 B: slave, 0x41,                     [hstatus:3|hseq:5], resp8, CRC, CRC
 *
 * This header is deliberately free of AVR and firmware dependencies so the
 * host test can compile the validation table directly (see test/cq_test.c).
 */

#include <stdint.h>
#include <stdbool.h>

#define FC_QUEUE             0x41u

// Request lengths, INCLUDING the CRC.
#define FCQ_REQ_POLL         5u   // slave, fc, [flags|ack], crc x2 — ack and/or poll
#define FCQ_REQ_ENQ          7u   // ...+ addr, data                — enqueue
#define FCQ_REQ_ENQ_DTR0     8u   // ...+ addr, data, dtr0          — enqueue w/ preload

// Flags in bits 7-5 of the request's third byte. These are the WIRE positions,
// and they are NOT the internal QF_* command bits the DALI adapter tests — the
// parser normalises between them (see queue_flags_to_qf in modbus.c).
#define QW_TWICE             0x80u   // send the forward frame twice; no answer expected
#define QW_DTR0              0x40u   // emit 0xA3 <dtr0> before the frame; dtr0 byte present
#define QW_ANSWER            0x20u   // a backward frame is expected; capture the answer
#define QW_FLAGS_bm          0xE0u

// Accept status, the top 3 bits of the reply's accept byte.
#define A_ACCEPTED           0u   // enqueued; aseq (1..31) is the command's identity
#define A_FULL               1u   // not enqueued, aseq = 0

// Is this (length, flags) pair a legal FC 0x41 request?
//
//   5 B  ack and/or poll, no enqueue   flags must be 000
//   7 B  enqueue, no DTR0 preload      QW_DTR0 must be CLEAR
//   8 B  enqueue with DTR0 preload     QW_DTR0 must be SET
//
// Anything else — any other length, or a length/flag inconsistency — is false,
// and the caller discards the frame WITHOUT A REPLY exactly as it would a
// corrupt one. To the master the two are indistinguishable and share the one
// recovery path, which is why there is no exception code for this.
//
// THIS IS THE SECURITY-RELEVANT CHECK, not just a tidiness one: the DTR0 flag is
// what decides whether byte 5 is a payload byte or the first CRC byte. Accepting
// an 8-byte frame with QW_DTR0 clear would feed a CRC byte to the ballast as a
// DTR0 value; accepting a 7-byte frame with it set would read the CRC as payload
// and the payload as CRC. A crafted length must not be able to walk through
// here, so the table is exhaustive and the default is reject.
//
// `flags` is the whole third byte or just its flag bits — the ack sequence in
// the low 5 is masked off here, so callers need not pre-mask. It is a pure
// function of its two arguments precisely so the host test can enumerate the
// entire input space against it (test/cq_test.c), which is the only way this
// table gets exercised: the bench has no independent RS-485 master to inject
// malformed frames with.
static inline bool fc41_len_ok(uint8_t len, uint8_t flags) {
    flags &= QW_FLAGS_bm;
    if (len == FCQ_REQ_POLL)     return flags == 0u;
    if (len == FCQ_REQ_ENQ)      return (flags & QW_DTR0) == 0u;
    if (len == FCQ_REQ_ENQ_DTR0) return (flags & QW_DTR0) != 0u;
    return false;
}

#endif /* FC41_H */
