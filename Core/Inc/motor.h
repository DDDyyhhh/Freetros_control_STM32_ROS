// Motor.h

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h" // 包含 CubeMX 生成的主头文件
#include <stdint.h> // 包含标准整数类型定义

// 定义电机编号，方便上层逻辑调用，增加可读性
#define MOTOR_FRONT_LEFT   0
#define MOTOR_FRONT_RIGHT  1
#define MOTOR_REAR_LEFT    2
#define MOTOR_REAR_RIGHT   3

/**
  * @brief  初始化所有电机相关的PWM和GPIO
  * @param  None
  * @retval None
  */
void Motor_Init(void);

/**
  * @brief  设置单个电机的速度和方向
  * @param  motor_index: 电机编号 (0-3), 推荐使用上面的宏定义
  * @param  speed: PWM值，范围由你的定时器 ARR 决定 (例如 -1000 ~ 1000)
  * @retval None
  */
void Motor_SetSpeed(uint8_t motor_index, int16_t speed);

void Motor_SetSpeed_Right(int16_t Speed);
void Motor_SetSpeed_Left(int16_t Speed);

#endif /* __MOTOR_H */