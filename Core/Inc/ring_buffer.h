#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buffer;
    uint16_t head;
    uint16_t tail;
    uint16_t size;
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size);
bool RingBuffer_Write(RingBuffer_t *rb, uint8_t data);
bool RingBuffer_Read(RingBuffer_t *rb, uint8_t *data);

#endif