#include <stdio.h>
#include <stdlib.h>
#include <string.h>


typedef struct {
    uint64_t ts;  // Display time (ms since epoch)
    int32_t rgbs[200];  // pixel rgb values
} TreeFrame_t;


typedef struct {
    TreeFrame_t *buffer;
    unsigned head;
    unsigned tail;
    unsigned full;
    unsigned size;
} RingBuffer_t;


RingBuffer_t *ring_buffer_init(unsigned size);
void ring_buffer_destroy(RingBuffer_t *rb);
int ring_buffer_is_full(RingBuffer_t *rb);
int ring_buffer_is_empty(RingBuffer_t *rb);
int ring_buffer_numframes(RingBuffer_t *rb);

// Does a memory copy. 
void ring_buffer_put(RingBuffer_t *rb, TreeFrame_t *data);

TreeFrame_t ring_buffer_get(RingBuffer_t *rb);
TreeFrame_t *ring_buffer_peek(RingBuffer_t *rb);

