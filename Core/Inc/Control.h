// Control.h

#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"
#include "cmsis_os.h" // 引入FreeRTOS头文件

#define MOTOR_NUM 4 // 定义电机数量


// 从MaixCAM传来的坐标结构体
typedef struct {
    int16_t x;
    int16_t y;
} TargetCoord_t;

// 全局声明队列和互斥锁句柄
extern osMessageQId targetCoordQueueHandle;
extern osMutexId telemetryDataMutexHandle;


// 函数声明

// void Control_Loop(void);
// void Control_Set_Target_Position_Left(float ticks);
// void Control_Increase_Target_Speed_Ticks(float increment);
// void Control_Set_Target_Position_Right(float ticks);

// 对外提供的函数接口
void Control_Init(void);
void Vision_ReceiveAndParse(void); // 在visionTask中调用
void Control_PID_Loop(void);      // 在controlTask中调用
void Control_Telemetry_Loop(void); // 在telemetryTask中调用

#endif /* __CONTROL_H */

