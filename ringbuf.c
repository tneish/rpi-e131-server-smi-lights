#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

#include "ringbuf.h"

RingBuffer_t *ring_buffer_init(unsigned size) {
    RingBuffer_t *rb = calloc(1, sizeof(RingBuffer_t));
    rb->buffer = calloc(size, sizeof(TreeFrame_t));
    rb->head = 0;
    rb->tail = 0;
    rb->full = 0;
    rb->size = size;
    return rb;
}

int ring_buffer_is_full(RingBuffer_t *rb) {
    return rb->full;
}

int ring_buffer_is_empty(RingBuffer_t *rb) {
    return (!rb->full && (rb->head == rb->tail));
}

void ring_buffer_put(RingBuffer_t *rb, TreeFrame_t *data) {
    memcpy(&(rb->buffer[rb->head]), data, 
        sizeof(TreeFrame_t));
    rb->head = (rb->head + 1) % rb->size;

    if (rb->full) {
        rb->tail = (rb->tail + 1) % rb->size;
    }

    rb->full = (rb->head == rb->tail);
}

TreeFrame_t ring_buffer_get(RingBuffer_t *rb) {
    if (ring_buffer_is_empty(rb)) {
        printf("Buffer empty!\n");
        assert(0);
    }

    TreeFrame_t data;
    memcpy(&data, &(rb->buffer[rb->tail]), 
        sizeof(TreeFrame_t));
    
    rb->tail = (rb->tail + 1) % rb->size;
    rb->full = 0;

    return data;
}

TreeFrame_t *ring_buffer_peek(RingBuffer_t *rb) {
    if (ring_buffer_is_empty(rb)) {
        printf("Buffer empty!\n");
        assert(0);
    }
    
    return &(rb->buffer[rb->tail]);
    
}

int ring_buffer_numframes(RingBuffer_t *rb) {
    if (rb->head > rb->tail) {
        return rb->head - rb->tail;
    } else {
        return rb->size - (rb->tail - rb->head);
    }
   
}

void ring_buffer_destroy(RingBuffer_t *rb) {
    if (rb->buffer) free(rb->buffer);
    free(rb);
    return;
}



