#include <stdlib.h>
#include "queue.h"
#include <string.h>


queue_result_t queue_init(queue_t *q, uint8_t queue_sz, size_t item_sz) {
    q->size = queue_sz;
    q->head = 0;
    q->tail = 0;
    q->item_sz = item_sz;
    q->entries = malloc(queue_sz * item_sz);
    if (!q->entries) {
        return QUEUE_NOMEM;
    }
    return QUEUE_OK;
}


queue_result_t queue_push(queue_t *q, void *data) {
    if ((q->tail+1) % q->size == q->head) {
        return QUEUE_FULL;
    }
    memcpy(q->entries + q->tail*q->item_sz, data, q->item_sz);
    q->tail = (q->tail + 1) % q->size;
    return QUEUE_OK;
}

queue_result_t queue_pop(queue_t *q, void  *out) {
    if (q->tail == q->head) {
        return QUEUE_EMPTY;
    }
    memcpy(out, q->entries + q->head*q->item_sz, q->item_sz);
    q->head = (q->head + 1) % q->size;
    return QUEUE_OK;
}