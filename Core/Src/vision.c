#include "vision.h"
#include "Control.h" // 需要用到 TargetCoord_t
#include "usart.h"   // 需要用到 huart3
#include "ring_buffer.h" // <<< 新增
#include <string.h>  // 用于 memcpy, strstr
#include <stdlib.h>  // 用于 atoi
#include <stdio.h>  // <<< 新增这一行
#include "Serial.h"
// --- 环形缓冲区 ---
#define VISION_RING_BUFFER_SIZE 256
static uint8_t vision_ring_buffer_data[VISION_RING_BUFFER_SIZE];
static RingBuffer_t vision_rb;

// --- 信号量 ---
// 在 freertos.c 中定义，这里 extern 声明
extern osSemaphoreId visionSemHandle; 

// DMA 接收缓冲区 (现在可以小一点)
#define VISION_DMA_BUFFER_SIZE 64
uint8_t vision_dma_buffer[VISION_DMA_BUFFER_SIZE];

void Vision_Init(void) {
    RingBuffer_Init(&vision_rb, vision_ring_buffer_data, VISION_RING_BUFFER_SIZE);
    // 【重要】启动单字节接收中断，而不是DMA
    HAL_UART_Receive_IT(&huart3, vision_dma_buffer, 1); 
}

void Vision_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) { // <<< 这里改成你的MaixCAM串口
        RingBuffer_Write(&vision_rb, vision_dma_buffer[0]);
        if (vision_dma_buffer[0] == '\n') {
            osSemaphoreRelease(visionSemHandle);
        }
        HAL_UART_Receive_IT(&huart3, vision_dma_buffer, 1);
    }
}

// 【新增】Vision_ProcessData 函数，由 visionTask 调用
void Vision_ProcessData(void)
{
    static uint8_t packet_buffer[128];
    static uint16_t packet_len = 0;
    uint8_t byte;

    // 从环形缓冲区读取所有字节
    while (RingBuffer_Read(&vision_rb, &byte))
    {
        if (byte == 'S') {
            packet_len = 0; // 遇到帧头，重新开始
        }
        
        if (packet_len < sizeof(packet_buffer) - 1) {
            packet_buffer[packet_len++] = byte;
        }

        if (byte == '\n') {
            packet_buffer[packet_len] = '\0'; // 添加字符串结束符
            
            // --- 开始解析 ---
            int x, y;
            if (sscanf((const char*)packet_buffer, "S,%d,%d,E\n", &x, &y) == 2)
            {
                TargetCoord_t coord_msg;
                coord_msg.x = x;
                coord_msg.y = y;
                osMessagePut(targetCoordQueueHandle, *(uint32_t*)&coord_msg, 0);
            }
            packet_len = 0; // 解析完清零
        }
    }
}