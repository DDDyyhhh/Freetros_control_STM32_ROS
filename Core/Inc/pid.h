// PID.h

#ifndef __PID_H
#define __PID_H

#include <stdint.h> // 包含标准整数类型

// PID 结构体定义 (与原版一致，使用浮点数)
typedef struct {
	float Target;     // 目标值
	float Actual;     // 实际值
	float Out;        // PID 输出值

	float Kp;         // 比例系数
	float Ki;         // 积分系数
	float Kd;         // 微分系数

	float Error0;     // 本次误差
	float Error1;     // 上次误差
	float ErrorInt;   // 误差积分

	float OutMax;     // 输出限幅最大值
	float OutMin;     // 输出限幅最小值

	// 【新增】积分分离阈值
    float IntegralSeparationThreshold;
} PID_t;

// 函数声明
void PID_Init(PID_t *p, float Kp, float Ki, float Kd, float OutMax, float OutMin);
float PID_Update(PID_t *p, float Target, float Actual);
void PID_Reset(PID_t *p);

#endif /* __PID_H */