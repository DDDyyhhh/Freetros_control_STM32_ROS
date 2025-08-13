// Encoder.c
#include "Encoder.h"
#include "main.h"



// ======================= 【硬件定时器宏定义 - 非常重要！】 =======================
// 在这里将你的 CubeMX 定时器配置与逻辑编号对应起来
// 请根据你的CubeMX配置仔细填写！
// 假设：
//   编码器0 (左前) -> TIM1
//   编码器1 (右前) -> TIM2
//   编码器2 (左后) -> TIM4
//   编码器3 (右后) -> TIM5

// 外部引用你在 CubeMX 中为编码器模式配置好的定时器句柄
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

// 将定时器句柄与逻辑编号绑定
#define ENCODER0_TIM_HANDLE   htim1  // 编码器0 (左前)
#define ENCODER1_TIM_HANDLE   htim2  // 编码器1 (右前)
#define ENCODER2_TIM_HANDLE   htim4  // 编码器2 (左后)
#define ENCODER3_TIM_HANDLE   htim5  // 编码器3 (右后)

// ==============================================================================

/**
  * @brief  初始化所有编码器接口
  * @note   该函数应该在系统初始化时被调用一次
  */
void Encoder_Init(void)
{
    // 启动所有4个编码器接口的定时器
    HAL_TIM_Encoder_Start(&ENCODER0_TIM_HANDLE, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENCODER1_TIM_HANDLE, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENCODER2_TIM_HANDLE, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENCODER3_TIM_HANDLE, TIM_CHANNEL_ALL);
}

/**
  * @brief  获取单个编码器在一次控制周期内的增量值，并清零计数器
  * @param  encoder_index: 编码器编号 (0-3)
  * @retval int32_t: 编码器脉冲增量 (有符号，表示方向)
  */
int32_t Encoder_Get_Increment(uint8_t encoder_index)
{
    int16_t increment = 0;

    // 根据编码器编号选择对应的定时器
    switch(encoder_index)
    {
        case ENCODER_FRONT_LEFT: //  编码器0 (TIM1 - 16位)
            // 读取计数器值，强制转换为有符号16位，可以巧妙处理正反转溢出
            increment = (int16_t)__HAL_TIM_GET_COUNTER(&ENCODER0_TIM_HANDLE);
            // 读取后立即清零，为下一个周期做准备
            __HAL_TIM_SET_COUNTER(&ENCODER0_TIM_HANDLE, 0);
            break; 
            
        case ENCODER_FRONT_RIGHT: // 编码器1 (TIM2 - 32位)
            increment = (int32_t)__HAL_TIM_GET_COUNTER(&ENCODER1_TIM_HANDLE);
            __HAL_TIM_SET_COUNTER(&ENCODER1_TIM_HANDLE, 0);
            break;
            
        case ENCODER_REAR_LEFT: /// 编码器2 (TIM4 - 16位)
            increment = (int16_t)__HAL_TIM_GET_COUNTER(&ENCODER2_TIM_HANDLE);
            __HAL_TIM_SET_COUNTER(&ENCODER2_TIM_HANDLE, 0);
            break;
            
        case ENCODER_REAR_RIGHT: // 编码器3
            increment = (int32_t)__HAL_TIM_GET_COUNTER(&ENCODER3_TIM_HANDLE);
            __HAL_TIM_SET_COUNTER(&ENCODER3_TIM_HANDLE, 0);
            break;
            
        default:
            // 无效编号，返回0
            increment = 0;
            break;
    }

    return increment;
}

