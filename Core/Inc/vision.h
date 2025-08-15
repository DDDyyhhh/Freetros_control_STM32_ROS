#ifndef INC_VISION_H_
#define INC_VISION_H_

#include "main.h"

// 定义接收缓冲区的最大长度
#define VISION_RX_BUFFER_SIZE 64

// 初始化视觉串口接收
void Vision_Init(void);

// 这个函数将在 visionTask 中被循环调用
void Vision_ProcessData(void);

void Vision_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#endif /* INC_VISION_H_ */