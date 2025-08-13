// Encoder.h
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

// 注意：这里的编号应该与 Motor.h 中的电机编号保持一致！
// 这样 motor[i] 就对应 encoder[i]
#define ENCODER_FRONT_LEFT   0
#define ENCODER_FRONT_RIGHT  1
#define ENCODER_REAR_LEFT    2
#define ENCODER_REAR_RIGHT   3

/**
  * @brief  初始化所有编码器接口
  * @param  None
  * @retval None
  */
void Encoder_Init(void);

/**
  * @brief  获取单个编码器在一次控制周期内的增量值，并清零计数器
  * @param  encoder_index: 编码器编号 (0-3)，推荐使用上面的宏定义
  * @retval int32_t: 编码器脉冲增量 (有符号，表示方向)
  */
int32_t Encoder_Get_Increment(uint8_t encoder_index);



#endif /* __ENCODER_H */