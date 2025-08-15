// Motor.c

#include "Motor.h"

// 外部引用你在 CubeMX 中配置好的定时器句柄
extern TIM_HandleTypeDef htim3; // 左右轮 PWM
extern TIM_HandleTypeDef htim1; // 左上轮编码器
extern TIM_HandleTypeDef htim2; // 右上轮编码器
extern TIM_HandleTypeDef htim4; // 左下轮编码器
extern TIM_HandleTypeDef htim5; // 右下轮编码器
#define MOTOR_DEADBAND 6 // 这个值需要实验测定！范围可能是20-50

// ======================= 【硬件引脚宏定义 - 非常重要！】 =======================
// 在这里将你的物理连接和逻辑编号对应起来
// 请根据你的CubeMX配置和电路板接线，仔细填写这里的每一个宏！
// 假设：
//   电机0 (左前)
//   电机1 (右前)
//   电机2 (左后)
//   电机3 (右后)

// --- 电机0 (左前) ---
#define MOTOR0_PWM_CHANNEL      TIM_CHANNEL_1   // 假设用 TIM3_CH1 (PA6)
// --- 电机1 (右前) ---
#define MOTOR1_PWM_CHANNEL      TIM_CHANNEL_2   // 假设用 TIM3_CH2 (PA7)
// --- 电机2 (左后) ---
#define MOTOR2_PWM_CHANNEL      TIM_CHANNEL_3   // 假设用 TIM3_CH3 (PB0)
// --- 电机3 (右后) ---
#define MOTOR3_PWM_CHANNEL      TIM_CHANNEL_4   // 假设用 TIM3_CH4 (PB1)
// ==============================================================================
void Motor_Init(void)
{
    // 启动 PWM 通道
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // 启动 PA6 (左上轮 PWM)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // 启动 PA7 (右上轮 PWM)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // 启动 PB0 (左下轮 PWM)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // 启动 PB1 (右上轮 PWM)
    
    // 启动编码器接口
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 【修正】启动 TIM4 (右轮编码器)
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // 【修正】启动 TIM2 (左轮编码器)
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 【修正】启动 TIM4 (右轮编码器)
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // 【修正】启动 TIM2 (左轮编码器)
}


/**
  * @brief  设置单个电机的速度和方向
  * @param  motor_index: 电机编号 (0-3)
  * @param  speed: PWM值。注意：其最大值应与TIM3的ARR寄存器值匹配
  */
void Motor_SetSpeed(uint8_t motor_index, int16_t speed)
{
    // 假设你的TIM3 ARR值为100，那么速度范围就是 -1000 ~ 1000
    const int16_t MAX_PWM = 99; 

    // 1. 限幅，防止传入过大的值
    if (speed > MAX_PWM)  speed = MAX_PWM;
    if (speed < -MAX_PWM) speed = -MAX_PWM;

    // 2. 根据电机编号选择对应的引脚和通道
    GPIO_TypeDef* in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef* in2_port;
    uint16_t      in2_pin;
    uint32_t      pwm_channel;

    switch(motor_index)
    {
        case MOTOR_FRONT_LEFT: // 电机0
            in1_port = AIN1_GPIO_Port; in1_pin = AIN1_Pin;
            in2_port = AIN2_GPIO_Port; in2_pin = AIN2_Pin;
            pwm_channel = MOTOR0_PWM_CHANNEL;
            break;
            
        case MOTOR_FRONT_RIGHT: // 电机1
            in1_port = BIN1_GPIO_Port; in1_pin = BIN1_Pin;
            in2_port = BIN2_GPIO_Port; in2_pin = BIN2_Pin;
            pwm_channel = MOTOR1_PWM_CHANNEL;
            break;
            
        case MOTOR_REAR_LEFT: // 电机2
            in1_port = CIN1_GPIO_Port; in1_pin = CIN1_Pin;
            in2_port = CIN2_GPIO_Port; in2_pin = CIN2_Pin;
            pwm_channel = MOTOR2_PWM_CHANNEL;
            break;
            
        case MOTOR_REAR_RIGHT: // 电机3
            in1_port = DIN1_GPIO_Port; in1_pin = DIN1_Pin;
            in2_port = DIN2_GPIO_Port; in2_pin = DIN2_Pin;
            pwm_channel = MOTOR3_PWM_CHANNEL;
            break;
            
        default:
            return; // 无效的电机编号，直接返回
    }


    // 【新增】死区补偿逻辑
    if (speed > 0 && speed < MOTOR_DEADBAND)
    {
        speed = MOTOR_DEADBAND; // 如果想让它动，就给一个最小启动PWM
    }
    else if (speed < 0 && speed > -MOTOR_DEADBAND)
    {
        speed = -MOTOR_DEADBAND;
    }

    // 3. 根据速度正负设置方向引脚电平，并设置PWM占空比
    // 【注意】这里的 SET/RESET 组合决定了电机的正转方向，
    // 如果实际方向与预期相反，交换 in1 和 in2 的 SET/RESET 即可。
    if (speed >= 0) { // 正转
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, pwm_channel, speed);
    } else { // 反转
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, pwm_channel, -speed); // PWM值始终为正
    }
}



