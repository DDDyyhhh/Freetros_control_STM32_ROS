// PID.c

#include "PID.h"
#include <math.h>
/**
  * @brief  初始化一个 PID 控制器结构体
  * @param  p 指向 PID 结构体的指针
  * @param  Kp, Ki, Kd PID 三个参数
  * @param  OutMax, OutMin 输出限幅
  * @retval None
  */
void PID_Init(PID_t *p, float Kp, float Ki, float Kd, float OutMax, float OutMin)
{
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->OutMax = OutMax;
    p->OutMin = OutMin;
    // p->IntegralSeparationThreshold = Threshold; // <<< 为新成员赋值

    // 清零所有内部状态变量
    PID_Reset(p);
}

/**
  * @brief  重置 PID 控制器的内部状态
  * @param  p 指向 PID 结构体的指针
  * @retval None
  */
void PID_Reset(PID_t *p)
{
    p->Target = 0.0f;
    p->Actual = 0.0f;
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->ErrorInt = 0.0f;
    p->Out = 0.0f;
}

/**
  * @brief  更新 PID 计算 (位置式 PID)
  * @param  p 指向 PID 结构体的指针
  * @param  Target 目标值
  * @param  Actual 实际值
  * @retval 计算后的 PID 输出值
  */
float PID_Update(PID_t *p, float Target, float Actual)
{
    // 更新目标值和实际值
    p->Target = Target;
    p->Actual = Actual;

    // 计算误差
    p->Error1 = p->Error0;					// 将上次误差保存
    p->Error0 = p->Target - p->Actual;		// 计算本次误差

    // 【1. 先累加积分】这是位置式PID的标准做法
    p->ErrorInt += p->Error0;
    // // 积分项计算 (带抗积分饱和，但原版逻辑更简单，我们先保持一致)
    // // 如果 Ki 不为0，才进行误差积分
    // if (fabs(p->Error0) < p->IntegralSeparationThreshold)
    // {
    //     p->ErrorInt += p->Error0;
    //     // 这里可以添加积分限幅来防止积分饱和
    //     // if (p->ErrorInt > IntegralLimit) p->ErrorInt = IntegralLimit;
    //     // if (p->ErrorInt < -IntegralLimit) p->ErrorInt = -IntegralLimit;
    // }
    // else
    // {
    //     p->ErrorInt = 0; // 如果 Ki 为 0，则清空积分项
    // }

    // 计算 PID 输出
    p->Out = p->Kp * p->Error0                           // P
           + p->Ki * p->ErrorInt                         // I
           + p->Kd * (p->Error0 - p->Error1);            // D

    // 【3. 积分抗饱和处理】
    // 如果计算出的总输出超出了限幅
    if (p->Out > p->OutMax)
    {
        // 并且当前误差的方向会继续加剧饱和（误差为正，想让输出更大）
        if(p->Error0 > 0)
        {
            // 那么就取消本次的积分累加
            p->ErrorInt -= p->Error0;
        }
        // 最后，将输出强制限制在最大值
        p->Out = p->OutMax;
    }
    else if (p->Out < p->OutMin)
    {
        // 同理，处理下限饱和
        if(p->Error0 < 0)
        {
            p->ErrorInt -= p->Error0;
        }
        p->Out = p->OutMin;
    }



    // 输出限幅
    if (p->Out > p->OutMax)
    {
        p->Out = p->OutMax;
    }
    if (p->Out < p->OutMin)
    {
        p->Out = p->OutMin;
    }

    // 返回最终的输出值
    return p->Out;
}