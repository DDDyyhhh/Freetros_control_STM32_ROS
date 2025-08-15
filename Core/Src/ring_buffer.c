#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size) {
    rb->buffer = buffer;
    rb->head = 0;
    rb->tail = 0;
    rb->size = size;
}

bool RingBuffer_Write(RingBuffer_t *rb, uint8_t data) {
    uint16_t next_head = (rb->head + 1) % rb->size;
    if (next_head == rb->tail) {
        return false; // Buffer is full
    }
    rb->buffer[rb->head] = data;
    rb->head = next_head;
    return true;
}

bool RingBuffer_Read(RingBuffer_t *rb, uint8_t *data) {
    if (rb->head == rb->tail) {
        return false; // Buffer is empty
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    return true;
}