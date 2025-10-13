#ifndef __MYQUEUE_H__
#define __MYQUEUE_H__

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t size;
    uint8_t head;
    uint8_t tail;
    uint8_t item_sz;
    void *entries;
} queue_t;

typedef enum {
    QUEUE_OK = 0,
    QUEUE_FULL,
    QUEUE_EMPTY,
    QUEUE_NOMEM,
} queue_result_t;

extern queue_result_t queue_init(queue_t *q, uint8_t queue_sz, size_t item_sz);
extern queue_result_t queue_push(queue_t *q, void *event);
extern queue_result_t queue_pop(queue_t *q, void *out);


#endif