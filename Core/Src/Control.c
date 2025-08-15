// Control.c (使用新单位和封装的 PID 库)

#include "Control.h"
#include "main.h"
#include "PID.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include <math.h>
#include <stdbool.h>
#include "FreeRTOS.h"
// ==================== 【调试模式切换开关】 ====================
// 模式 1: 调试内环 (速度环)
// 模式 2: 调试外环 (位置环)，此时内环参数应已调好
#define CONTROL_MODE 6  // <<< 在这里切换模式：0 测试4个电机是否正常 1调试单轮速度环 或 2调试四轮速度环 或 3无  或 4 测试视觉识别通信 6 视觉伺服调试模式

#define VISION_DEADBAND_X 10   // X方向误差在 ±5 像素内，就认为已经对准，停止旋转
#define VISION_DEADBAND_Y 10  // Y方向误差在 ±10 像素内，就认为距离合适，停止前进/后退

#define MAX_LINEAR_SPEED_CMPS 50.0f   // 限制最大前进/后退速度为 25 cm/s
#define MAX_ANGULAR_SPEED_RADPS 1.0f  // 限制最大旋转速度为 1.0 rad/s (~57度/秒)

// 超时阈值，单位：毫秒 (ms)。100ms 约等于3帧的间隔，是一个不错的起点
#define COORD_TIMEOUT_MS 100 
// ==========================================================
// ======================= 全局变量及常量定义 =======================

// --- PID 实例 ---
PID_t pid_speed[MOTOR_NUM]; // 为4个电机创建PID实例数组

// --- 物理参数 (需要为麦克纳姆轮重新标定!) ---
// 这些参数对运动的准确性至关重要
const float WHEEL_DIAMETER_CM = 7.5f;       // 麦轮直径 (cm)
const float WHEEL_BASE_CM = 20.0f;          // 左右轮距 (cm)
const float TRACK_WIDTH_CM = 22.0f;         // 前后轮距 (cm)
const float ENCODER_TICKS_PER_REV = 1456.0f;   // 编码器每转一圈的脉冲数 (减速后)
const float CONTROL_PERIOD_S = 0.01f;       // 控制周期 (秒, 10ms)

// --- 根据物理参数计算出的转换系数 ---
// 每 tick 对应的距离(cm) = (轮子周长) / (每转tick数)
const float CM_PER_TICK = (3.1415926f * WHEEL_DIAMETER_CM) / ENCODER_TICKS_PER_REV;

// --- 视觉伺服控制参数 (需要调试) ---
const int16_t IMAGE_CENTER_X = 112;     // 假设图像中心X坐标
const int16_t TARGET_Y_COORD = 112;     // 希望垃圾最终停在图像的这个Y坐标上 (表示距离合适)
const float KP_ANGULAR = 0.08f;         // 视觉旋转控制P增益
const float KP_LINEAR = 0.45f;           // 视觉前进控制P增益

// --- 共享的调试数据 (由互斥锁保护) ---
float g_target_speeds_cmps[MOTOR_NUM] = {0};
float g_current_speeds_cmps[MOTOR_NUM] = {0};
int16_t g_pwm_out[MOTOR_NUM] = {0};

uint8_t current_motor;
int16_t TEST_PWM = 6; // 使用一个较小的PWM值进行测试

uint8_t motor_to_read;
int32_t encoder_val; // 使用 int32_t 以匹配 Encoder_Get_Increment 的返回值

// --- 【新增】在文件顶部定义全局变量 ---
int16_t g_latest_coord_x = -1; // 初始值为-1，表示未收到
int16_t g_latest_coord_y = -1;

static float filtered_speed_0 = 0.0f;

// --- 【新增】用于平滑滤波的静态变量 ---
static float smoothed_vx = 0.0f;
static float smoothed_vw = 0.0f;

// --- 【新增】目标坐标超时处理 ---
// 使用 FreeRTOS 的 Tick Count 来做时间戳
static uint32_t last_coord_receive_time = 0; 
// 用于存储上一次有效坐标的静态变量
static TargetCoord_t last_valid_coord = {-1, -1};


//---------------------------------------------------------------------------------------------------

// 全局 PID 实例
PID_t pid_speed_right;
PID_t pid_speed_left;
PID_t pid_position_left; // <<<【新增】左轮的位置环 PID
PID_t pid_position_right; // <<<【新增】右轮的位置环 PID

// 全局目标速度 (单位: ticks/10ms)
static float g_target_ticks_left = 0.0f;
static float g_target_position_left = 0.0f;  // 目标位置 (单位: ticks)
static float g_current_position_left = 0.0f; // 当前位置 (单位: ticks
static float g_target_position_right = 0.0f;  // 目标位置 (单位: ticks)
static float g_current_position_right = 0.0f; // 当前位置 (单位: ticks


/**
  * @brief  初始化所有控制相关的变量
  */
void Control_Init(void)
{

	//**********************调试模式************************

	const int16_t MAX_PWM = 99; // 与你的Motor_SetSpeed中的MAX_PWM保持一致

    // --- 【新】积分分离阈值 ---
    const float SEPARATION_THRESHOLD = 0.0f; // 目标速度20的15%
    // 使用循环为4个电机初始化PID控制器
    for (int i = 0; i < MOTOR_NUM; i++) {
        // 初始PID参数，需要重新调试
        // Kp, Ki, Kd 的单位现在是针对 (cm/s) 的误差
        // OutMax/OutMin 的单位是 PWM 值
        PID_Init(&pid_speed[i], 1.5f, 0.1f, 0.02f, MAX_PWM, -MAX_PWM);
        // PID_Init(&pid_speed[i], 1.5f, 0.1f, 0.8f, MAX_PWM, -MAX_PWM,SEPARATION_THRESHOLD);
    }
		
		//*******************************************************************

}

// ======================= 工具函数 =======================

/**
  * @brief 根据编码器脉冲数计算轮子线速度 (cm/s)
  */
static inline float TicksToSpeedCMPS(int32_t ticks)
{
    return (float)ticks * CM_PER_TICK / CONTROL_PERIOD_S;
}

// ======================= RTOS 任务调用的核心函数 =======================

/**
  * @brief  视觉数据接收与解析函数 - 【在 visionTask 中被调用】
  */
void Vision_ReceiveAndParse(void)
{
    // --------------------------------------------------------
    // TODO: 实现你的串口接收和解析逻辑
    // --------------------------------------------------------
    // 这是一个占位符实现，你需要用真实的串口数据处理代码替换它
    
    // 模拟接收到了数据
    static uint32_t fake_counter = 0;
    fake_counter++;
    bool new_coord_received = (fake_counter % 10 == 0); // 模拟每10次调用收到一次数据

    if (new_coord_received) { 
        // 模拟一个从左到右移动的目标
        TargetCoord_t received_coord;
        received_coord.x = (fake_counter % 320);
        received_coord.y = 100;

        // 将解析出的坐标发送到队列
        osMessagePut(targetCoordQueueHandle, *(uint32_t*)&received_coord, 0);
    }
}


/**
  * @brief  主控制与PID循环 - 【在 controlTask 中被调用】
  */
void Control_PID_Loop(void)
{
    #if (CONTROL_MODE == 0)
      current_motor = g_test_motor_index;

      // --- 1. 测试正转 ---
      // 停止其他所有电机
      for (int i = 0; i < MOTOR_NUM; i++) {
          if (i != current_motor) {
              Motor_SetSpeed(i, 0);
          }
      }
      // 启动当前电机正转
      Serial_Printf("\r\n--> Testing Motor %d: FORWARD <--\r\n", current_motor);
      Motor_SetSpeed(current_motor, TEST_PWM);
      osDelay(3000); // 持续3秒

      // --- 2. 测试反转 ---
      Serial_Printf("--> Testing Motor %d: REVERSE <--\r\n", current_motor);
      Motor_SetSpeed(current_motor, -TEST_PWM);
      osDelay(3000); // 持续3秒

      // --- 3. 停止并准备切换到下一个电机 ---
      Serial_Printf("--> Testing Motor %d: STOP <--\r\n", current_motor);
      Motor_SetSpeed(current_motor, 0);
      osDelay(2000); // 休息2秒

      g_test_motor_index = (g_test_motor_index + 1) % MOTOR_NUM; // 切换到下一个电机
    

    #elif (CONTROL_MODE == 1)
    // ==================== 单轮PID调试 (只调试电机0) ====================
    
    // 1. 设定一个固定的目标速度给电机0 (单位: cm/s)
    // 你可以修改这个值来测试不同速度下的表现
    float target_speed_cmps = 20.0f; 
    
    // 2. 读取编码器，计算实际速度
    int32_t current_ticks_0 = Encoder_Get_Increment(MOTOR_FRONT_LEFT);
    float current_speed_cmps_0 = TicksToSpeedCMPS(current_ticks_0);

    // 一阶低通滤波
    // const float a = 0.7; // 滤波系数，a越接近1，滤波效果越强，但响应越慢
    // filtered_speed_0 = a * filtered_speed_0 + (1-a) * current_speed_cmps_0;

    // 3. 更新PID计算，得到PWM输出 (只对电机0的PID进行计算)
    int16_t pwm_out_0 = (int16_t)PID_Update(&pid_speed[MOTOR_FRONT_LEFT], target_speed_cmps, current_speed_cmps_0);
    
    // 4. 应用PWM到电机0
    Motor_SetSpeed(MOTOR_FRONT_LEFT, pwm_out_0);
    
    // 5. 其他电机必须停止，以排除干扰
    Motor_SetSpeed(MOTOR_FRONT_RIGHT, 0);
    Motor_SetSpeed(MOTOR_REAR_LEFT, 0);
    Motor_SetSpeed(MOTOR_REAR_RIGHT, 0);
    
    // 6. 使用互斥锁更新全局变量，供打印任务使用
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK)
    {
        g_target_speeds_cmps[MOTOR_FRONT_LEFT] = target_speed_cmps;
        g_current_speeds_cmps[MOTOR_FRONT_LEFT] = current_speed_cmps_0;
        g_pwm_out[MOTOR_FRONT_LEFT] = pwm_out_0;
        osMutexRelease(telemetryDataMutexHandle);
    }

    #elif (CONTROL_MODE == 2)
    // ==================== 四轮联动PID测试 ====================
    
    // --- 1. 手动设定机器人目标速度 (vx, vy, vw) ---
    // 我们来测试几种基本运动
    // vx: 前进速度(cm/s), vy: 平移速度(cm/s), vw: 旋转速度(rad/s)
    
    // 【测试指令】: 让机器人以 20 cm/s 的速度前进
    float target_vx_cmps = 0.0f;
    float target_vy_cmps = 15.0f;
    float target_vw_radps = 0.0f;
    
    // --- 2. 麦克纳姆轮运动学逆解 ---
    // 将机器人速度分解为四个轮子的目标速度
    float l_plus_w = (WHEEL_BASE_CM + TRACK_WIDTH_CM) / 2.0f;
    float local_target_speeds[MOTOR_NUM];
    
    local_target_speeds[MOTOR_FRONT_LEFT]  = target_vx_cmps - target_vy_cmps - l_plus_w * target_vw_radps;
    local_target_speeds[MOTOR_FRONT_RIGHT] = target_vx_cmps + target_vy_cmps + l_plus_w * target_vw_radps;
    local_target_speeds[MOTOR_REAR_LEFT]   = target_vx_cmps + target_vy_cmps - l_plus_w * target_vw_radps;
    local_target_speeds[MOTOR_REAR_RIGHT]  = target_vx_cmps - target_vy_cmps + l_plus_w * target_vw_radps;

    // --- 3. 四轮独立PID闭环控制 (使用循环) ---
    int32_t local_current_ticks[MOTOR_NUM];
    float local_current_speeds[MOTOR_NUM];
    int16_t local_pwm_out[MOTOR_NUM];

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        local_current_ticks[i] = Encoder_Get_Increment(i);
        local_current_speeds[i] = TicksToSpeedCMPS(local_current_ticks[i]);
        local_pwm_out[i] = (int16_t)PID_Update(&pid_speed[i], local_target_speeds[i], local_current_speeds[i]);
        Motor_SetSpeed(i, local_pwm_out[i]);
    }
    
    // --- 4. 使用互斥锁更新全局调试变量 ---
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK) 
    {
        for (int i = 0; i < MOTOR_NUM; i++) {
            g_target_speeds_cmps[i] = local_target_speeds[i]; 
            g_current_speeds_cmps[i] = local_current_speeds[i];
            g_pwm_out[i] = local_pwm_out[i];
        }
        osMutexRelease(telemetryDataMutexHandle);
    }


    #elif (CONTROL_MODE == 3)
    TargetCoord_t target_coord = {-1, -1}; 
    bool target_visible = false;
    
    // === 1. 从队列获取最新的垃圾坐标 ===
    osEvent event = osMessageGet(targetCoordQueueHandle, 0); 
    if (event.status == osEventMessage) {
        target_coord = *(TargetCoord_t*)&event.value.v;
    }

    // 【新的判断逻辑】根据坐标值判断目标是否有效
    if (target_coord.x != -1 && target_coord.y != -1) {
        target_visible = true;
    } else {
        target_visible = false;
    }

    // 更新全局调试变量，无论是否有效都更新
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK)
    {
        g_latest_coord_x = target_coord.x;
        g_latest_coord_y = target_coord.y;
        osMutexRelease(telemetryDataMutexHandle);
    }

    // --- 2. 根据坐标计算机器人目标速度 (vx, vy, vw) ---
    float target_vx_cmps = 0; // 前进速度
    float target_vy_cmps = 0; // 平移速度 (麦轮可用)
    float target_vw_radps = 0; // 旋转速度

    if (target_visible) {
        // a. 计算旋转速度 vw: 让机器人对准垃圾中心
        float error_x = IMAGE_CENTER_X - target_coord.x;
        target_vw_radps = KP_ANGULAR * error_x;

        // b. 计算前进速度 vx: 让机器人靠近或远离垃圾，维持在合适距离
        float error_y = TARGET_Y_COORD - target_coord.y;
        target_vx_cmps = KP_LINEAR * error_y;
    }else {
        // 【关键】如果目标丢失，所有目标速度必须清零！
        target_vx_cmps = 0;
        target_vy_cmps = 0;
        target_vw_radps = 0;
    }
    // 如果目标不可见 (target_visible == false), 所有目标速度都为0，机器人会停下

    // --- 3. 麦克纳姆轮运动学逆解 ---
    // 将机器人速度 (vx, vy, vw) 分解为四个轮子的目标切线速度
    float l_plus_w = (WHEEL_BASE_CM + TRACK_WIDTH_CM) / 2.0f;
    float local_target_speeds[MOTOR_NUM];
    
    // 假设电机编号: 0-左前, 1-右前, 2-左后, 3-右后
    local_target_speeds[MOTOR_FRONT_LEFT]  = target_vx_cmps - target_vy_cmps - l_plus_w * target_vw_radps;
    local_target_speeds[MOTOR_FRONT_RIGHT] = target_vx_cmps + target_vy_cmps + l_plus_w * target_vw_radps;
    local_target_speeds[MOTOR_REAR_LEFT]   = target_vx_cmps + target_vy_cmps - l_plus_w * target_vw_radps; // 注意符号
    local_target_speeds[MOTOR_REAR_RIGHT]  = target_vx_cmps - target_vy_cmps + l_plus_w * target_vw_radps; // 注意符号

    // --- 4. 四轮独立PID闭环控制 ---
    int16_t local_current_ticks[MOTOR_NUM];
    float local_current_speeds[MOTOR_NUM];
    int16_t local_pwm_out[MOTOR_NUM];

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        // a. 读取当前编码器增量
        local_current_ticks[i] = Encoder_Get_Increment(i);
        // b. 计算实际速度
        local_current_speeds[i] = TicksToSpeedCMPS(local_current_ticks[i]);
        // c. 更新PID计算
        local_pwm_out[i] = (int16_t)PID_Update(&pid_speed[i], local_target_speeds[i], local_current_speeds[i]);
        // d. 应用到电机
        Motor_SetSpeed(i, local_pwm_out[i]);
    }
    
    // --- 5. 使用互斥锁更新全局调试变量 ---
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK) 
    {
        for (int i = 0; i < MOTOR_NUM; i++) {
            g_target_speeds_cmps[i] = local_target_speeds[i]; 
            g_current_speeds_cmps[i] = local_current_speeds[i];
            g_pwm_out[i] = local_pwm_out[i];
        }
        osMutexRelease(telemetryDataMutexHandle);
    }




    #elif (CONTROL_MODE == 4)// <<< 新增串口测试模式
        // 在这个模式下，我们不控制电机，只为了验证串口通信
        // 停止所有电机
        Motor_SetSpeed(MOTOR_FRONT_LEFT, 0);
        Motor_SetSpeed(MOTOR_FRONT_RIGHT, 0);
        Motor_SetSpeed(MOTOR_REAR_LEFT, 0);
        Motor_SetSpeed(MOTOR_REAR_RIGHT, 0);
        
        // controlTask 可以简单地延时一下
        osDelay(10);



    #elif (CONTROL_MODE == 5)
    // --- 1. 从队列获取最新的目标坐标 ---
    TargetCoord_t target_coord = {0, 0}; // 默认目标丢失
    bool target_visible = false;
    
    osEvent event = osMessageGet(targetCoordQueueHandle, 0); 
    if (event.status == osEventMessage) {
        target_coord = *(TargetCoord_t*)&event.value.v;
        target_visible = true;
    }

    // --- 2. 根据坐标计算机器人目标速度 (vx, vy, vw) ---
    float target_vx_cmps = 0; // 前进速度
    float target_vy_cmps = 0; // 平移速度 (麦轮可用)
    float target_vw_radps = 0; // 旋转速度

    if (target_visible) {
        // a. 计算旋转速度 vw: 让机器人对准垃圾中心
        float error_x = IMAGE_CENTER_X - target_coord.x;
        target_vw_radps = KP_ANGULAR * error_x;

        // b. 计算前进速度 vx: 让机器人靠近或远离垃圾，维持在合适距离
        float error_y = TARGET_Y_COORD - target_coord.y;
        target_vx_cmps = KP_LINEAR * error_y;
    }
    // 如果目标不可见 (target_visible == false), 所有目标速度都为0，机器人会停下

    // --- 3. 麦克纳姆轮运动学逆解 ---
    // 将机器人速度 (vx, vy, vw) 分解为四个轮子的目标切线速度
    float l_plus_w = (WHEEL_BASE_CM + TRACK_WIDTH_CM) / 2.0f;
    float local_target_speeds[MOTOR_NUM];
    
    // 假设电机编号: 0-左前, 1-右前, 2-左后, 3-右后
    local_target_speeds[MOTOR_FRONT_LEFT]  = target_vx_cmps - target_vy_cmps - l_plus_w * target_vw_radps;
    local_target_speeds[MOTOR_FRONT_RIGHT] = target_vx_cmps + target_vy_cmps + l_plus_w * target_vw_radps;
    local_target_speeds[MOTOR_REAR_LEFT]   = target_vx_cmps + target_vy_cmps - l_plus_w * target_vw_radps; // 注意符号
    local_target_speeds[MOTOR_REAR_RIGHT]  = target_vx_cmps - target_vy_cmps + l_plus_w * target_vw_radps; // 注意符号

    // --- 4. 四轮独立PID闭环控制 ---
    int16_t local_current_ticks[MOTOR_NUM];
    float local_current_speeds[MOTOR_NUM];
    int16_t local_pwm_out[MOTOR_NUM];

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        // a. 读取当前编码器增量
        local_current_ticks[i] = Encoder_Get_Increment(i);
        // b. 计算实际速度
        local_current_speeds[i] = TicksToSpeedCMPS(local_current_ticks[i]);
        // c. 更新PID计算
        local_pwm_out[i] = (int16_t)PID_Update(&pid_speed[i], local_target_speeds[i], local_current_speeds[i]);
        // d. 应用到电机
        Motor_SetSpeed(i, local_pwm_out[i]);
    }
    
    // --- 5. 使用互斥锁更新全局调试变量 ---
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK) 
    {
        for (int i = 0; i < MOTOR_NUM; i++) {
            g_target_speeds_cmps[i] = local_target_speeds[i]; 
            g_current_speeds_cmps[i] = local_current_speeds[i];
            g_pwm_out[i] = local_pwm_out[i];
        }
        osMutexRelease(telemetryDataMutexHandle);
    }


    #elif (CONTROL_MODE == 6)

    // --- 1. 初始化本周期的瞬时目标速度 ---
    // 默认情况下，我们认为本周期没有新的运动指令
    float instant_vx_cmps = 0;
    float instant_vw_radps = 0;
    float instant_vy_cmps = 0; // <<< 【新增】定义并初始化平移速度
    bool new_command_generated = false; // 标记本周期是否生成了新指令
    
     // --- 2. 尝试获取新坐标并更新状态 ---
    osEvent event = osMessageGet(targetCoordQueueHandle, 0); 
    if (event.status == osEventMessage) {
        TargetCoord_t new_coord = *(TargetCoord_t*)&event.value.v;
        // 无论收到什么，都先更新 last_valid_coord
        last_valid_coord = new_coord;

        if (new_coord.x != -1 && new_coord.y != -1) {
            // 如果是有效坐标，更新时间戳并标记有新指令
            last_coord_receive_time = osKernelSysTick();
            new_command_generated = true;
        }
    }

    // =========================================================
    // ===       【新增】在这里添加数据同步逻辑              ===
    // =========================================================
    // 更新全局调试变量，让 telemetryTask 能看到最新的坐标
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK)
    {
        // 无论收到的是有效坐标还是{-1,-1}，都更新到全局变量
        g_latest_coord_x = last_valid_coord.x;
        g_latest_coord_y = last_valid_coord.y;
        osMutexRelease(telemetryDataMutexHandle);
    }
    // =========================================================

    
    
    // --- 3. 超时判断与指令生成 ---
    // 如果距离上次收到有效数据的时间没有超时...
    if ((osKernelSysTick() - last_coord_receive_time) < COORD_TIMEOUT_MS) {
        // ...并且 last_valid_coord 是有效的
        if(last_valid_coord.x != -1) {
            
            // ==========================================================
            // ===            【在这里选择调试项目】                  ===
            // ==========================================================

            // --- 项目A: 只调试旋转 (对准目标) ---
            // const int16_t IMAGE_CENTER_X = 112; 
            // float error_x = IMAGE_CENTER_X - last_valid_coord.x;
            // if (fabs(error_x) > VISION_DEADBAND_X) {
            //     instant_vw_radps = KP_ANGULAR * error_x;
            // }
            // instant_vx_cmps = 0.0f; // 强制禁止前进

            
            // --- 项目B: 只调试前进/后退 (调整距离) ---
            const int16_t TARGET_Y_COORD = 112; 
            float error_y = TARGET_Y_COORD - last_valid_coord.y;
            if (fabs(error_y) > VISION_DEADBAND_Y) {
                instant_vx_cmps = KP_LINEAR * error_y;
            }
            instant_vw_radps = 0.0f; // 强制禁止旋转
            
        }
    }
    // 如果超时了，instant_vx/vw 将保持为0
    
    

    

    // --- 4. 对计算出的目标速度进行【限幅】 ---
    if (instant_vx_cmps > MAX_LINEAR_SPEED_CMPS) instant_vx_cmps = MAX_LINEAR_SPEED_CMPS;
    else if (instant_vx_cmps < -MAX_LINEAR_SPEED_CMPS) instant_vx_cmps = -MAX_LINEAR_SPEED_CMPS;

    if (instant_vw_radps > MAX_ANGULAR_SPEED_RADPS) instant_vw_radps = MAX_ANGULAR_SPEED_RADPS;
    else if (instant_vw_radps < -MAX_ANGULAR_SPEED_RADPS) instant_vw_radps = -MAX_ANGULAR_SPEED_RADPS;

    // --- 5. 对目标速度进行【平滑滤波】 ---
    const float SMOOTH_FACTOR = 0.7f; 
    smoothed_vx = SMOOTH_FACTOR * smoothed_vx + (1 - SMOOTH_FACTOR) * instant_vx_cmps;
    smoothed_vw = SMOOTH_FACTOR * smoothed_vw + (1 - SMOOTH_FACTOR) * instant_vw_radps;
    
    // --- 6. 使用【平滑后】的速度进行运动学逆解 ---
    float l_plus_w = (WHEEL_BASE_CM + TRACK_WIDTH_CM) / 2.0f;
    float local_target_speeds[MOTOR_NUM];
    
    local_target_speeds[MOTOR_FRONT_LEFT]  = smoothed_vx - instant_vy_cmps - l_plus_w * smoothed_vw;
    local_target_speeds[MOTOR_FRONT_RIGHT] = smoothed_vx + instant_vy_cmps + l_plus_w * smoothed_vw;
    local_target_speeds[MOTOR_REAR_LEFT]   = smoothed_vx + instant_vy_cmps - l_plus_w * smoothed_vw;
    local_target_speeds[MOTOR_REAR_RIGHT]  = smoothed_vx - instant_vy_cmps + l_plus_w * smoothed_vw;
    
    // --- 4. 四轮独立PID闭环控制 ---
    int16_t local_current_ticks[MOTOR_NUM];
    float local_current_speeds[MOTOR_NUM];
    int16_t local_pwm_out[MOTOR_NUM];

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        // a. 读取当前编码器增量
        local_current_ticks[i] = Encoder_Get_Increment(i);
        // b. 计算实际速度
        local_current_speeds[i] = TicksToSpeedCMPS(local_current_ticks[i]);
        // c. 更新PID计算
        local_pwm_out[i] = (int16_t)PID_Update(&pid_speed[i], local_target_speeds[i], local_current_speeds[i]);
        // d. 应用到电机
        Motor_SetSpeed(i, local_pwm_out[i]);
    }
    
    // --- 5. 使用互斥锁更新全局调试变量 ---
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK) 
    {
        for (int i = 0; i < MOTOR_NUM; i++) {
            g_target_speeds_cmps[i] = local_target_speeds[i]; 
            g_current_speeds_cmps[i] = local_current_speeds[i];
            g_pwm_out[i] = local_pwm_out[i];
        }
        osMutexRelease(telemetryDataMutexHandle);
    }




    #endif
}


/**
  * @brief  串口遥测循环 - 【在 telemetryTask 中被调用】
  */
void Control_Telemetry_Loop(void)
{

  #if (CONTROL_MODE == 0)
    motor_to_read = g_test_motor_index;
    
    // 读取对应编码器的增量值
    encoder_val = Encoder_Get_Increment(motor_to_read);

    // 打印信息
    Serial_Printf("Motor %d, Encoder val: %ld\r\n", motor_to_read, encoder_val);
    
    

  #elif (CONTROL_MODE == 1)
  float local_target, local_current;
    int16_t local_pwm;
    
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK)
    {
        // 只读取电机0的数据
        local_target = g_target_speeds_cmps[MOTOR_FRONT_LEFT];
        local_current = g_current_speeds_cmps[MOTOR_FRONT_LEFT];
        local_pwm = g_pwm_out[MOTOR_FRONT_LEFT];
        osMutexRelease(telemetryDataMutexHandle);
        
        // 打印三个关键值，用逗号隔开，方便用串口示波器软件查看
        // 格式: 目标速度,实际速度,PWM输出
        Serial_Printf("%f,%f,%d\r\n", local_target, local_current, local_pwm);
    }
  
  #elif (CONTROL_MODE == 2)

  float local_targets[MOTOR_NUM];
    float local_currents[MOTOR_NUM];
    
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK)
    {
        for (int i = 0; i < MOTOR_NUM; i++) {
            local_targets[i] = g_target_speeds_cmps[i];
            local_currents[i] = g_current_speeds_cmps[i];
        }
        osMutexRelease(telemetryDataMutexHandle);
        
        // 打印电机0和电机1的目标与实际速度，方便对比
        Serial_Printf("T0:%.2f,C0:%.2f,T1:%.2f,C1:%.2f\r\n", 
                      local_targets[MOTOR_FRONT_LEFT], local_currents[MOTOR_FRONT_LEFT],
                      local_targets[MOTOR_FRONT_RIGHT], local_currents[MOTOR_FRONT_RIGHT]);
    }

  #elif (CONTROL_MODE == 3)

   // --- 【关键修改】打印收到的坐标 ---
    int16_t x, y;

    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK)
    {
        // 从全局变量中拷贝最新的坐标值
        x = g_latest_coord_x;
        y = g_latest_coord_y;
        osMutexRelease(telemetryDataMutexHandle);
        
        // 打印出来，方便观察
        Serial_Printf("Received Coords -> X: %d, Y: %d\r\n", x, y);
    }
  #elif (CONTROL_MODE == 4)// <<< 新增串口测试模式
  int16_t x, y;
        if (osMutexWait(telemetryDataMutexHandle, 10) == osOK)
        {
            x = g_latest_coord_x;
            y = g_latest_coord_y;
            osMutexRelease(telemetryDataMutexHandle);
            Serial_Printf("Received Coords -> X: %d, Y: %d\r\n", x, y);
        }


  #elif (CONTROL_MODE == 6)// <<< 新增串口测试模式
  int16_t x, y;
        if (osMutexWait(telemetryDataMutexHandle, 10) == osOK)
        {
            x = g_latest_coord_x;
            y = g_latest_coord_y;
            osMutexRelease(telemetryDataMutexHandle);
            Serial_Printf("Received Coords -> X: %d, Y: %d\r\n", x, y);
        }
  
  #elif (CONTROL_MODE == 5)
    float local_targets[MOTOR_NUM];
    float local_currents[MOTOR_NUM];
    int16_t local_pwms[MOTOR_NUM];
    
    // --- 使用互斥锁安全地拷贝数据 ---
    if (osMutexWait(telemetryDataMutexHandle, 10) == osOK)
    {
        for (int i = 0; i < MOTOR_NUM; i++) {
            local_targets[i] = g_target_speeds_cmps[i];
            local_currents[i] = g_current_speeds_cmps[i];
            local_pwms[i] = g_pwm_out[i];
        }
        osMutexRelease(telemetryDataMutexHandle);
        
        // --- 打印数据 ---
        // 打印前两个轮子的数据用于调试
        Serial_Printf("T0:%.2f,C0:%.2f,P0:%d,T1:%.2f,C1:%.2f,P1:%d\r\n", 
                      local_targets[0], local_currents[0], local_pwms[0],
                      local_targets[1], local_currents[1], local_pwms[1]);
    }

    #endif
}





/**
  * @brief  主控制循环 - 【在 10ms 定时器中断中被调用】
  */
// void Control_Loop(void)
// {
//     // ============ 左轮 速度环PID 调试 (单位: ticks/10ms) ============

//     // 1. 读取由按键控制的全局目标速度
// //    float target_ticks_left = g_target_ticks_left;

// //    // 2. 读取左轮编码器增量，这就是我们的实际速度
// //    float current_ticks_left = (float)Encoder_Get_Left();

// //    // 3. 更新 PID 计算，得到 PWM 输出值
// //    int16_t pwm_out_left = (int16_t)PID_Update(&pid_speed_left, target_ticks_left, current_ticks_left);

// //    // 4. 应用到左轮电机
// //    Motor_SetSpeed_Left(pwm_out_left);
// //    
// //    // 5. 停止右轮电机
// //    Motor_SetSpeed_Right(0);
// //    
// //    // 6. 打印调试数据
// //    Serial_Printf("%f,%f,%d\r\n", target_ticks_left, current_ticks_left, pwm_out_left);





//    // 在循环开始时，只获取一次编码器读数
//     int32_t ticks_now_left = Encoder_Get_Increment(0);
//     int32_t ticks_now_right = Encoder_Get_Increment(1); // 即使不用，也先读出来保持对称

// #if (CONTROL_MODE == 1)
//     // ==================== 模式1：调试内环 (速度环) ====================
    
//     // 1. 设定一个固定的目标速度 (单位: ticks/10ms)
//     float target_speed_left = 10.0f;

//     // 2. 获取当前实际速度
//     float current_speed_left = (float)ticks_now_left;

//     // 3. 更新速度环 PID
//     int16_t pwm_out_left = (int16_t)PID_Update(&pid_speed_left, target_speed_left, current_speed_left);

//     // 4. 应用到电机
//     Motor_SetSpeed_Left(pwm_out_left);

//     // 5. 打印速度环的调试数据
//     // 格式: 目标速度, 实际速度, PWM输出
//     Serial_Printf("%f,%f,%d\r\n", target_speed_left, current_speed_left, pwm_out_left);


// #elif (CONTROL_MODE == 2)
//     // ==================== 模式2：调试外环 (位置环) ====================
//     // ** 前提：内环的速度 PID 参数已经调试到完美 **

//     // 1. 设定一个固定的目标位置 (单位: ticks)
//     //    你可以修改这个值来测试不同的定位距离
//     float target_position_left = 100.0f;

//     // --- 外环 (位置环) 计算 ---
//     // 2. 获取编码器增量，并累加得到当前位置
// //    g_current_position_left += (float)Encoder_Get_Left();
		
//     g_current_position_left += ticks_now_left;
		
    
//     // 3. 更新位置环 PID，其输出是【内环的目标速度】
//     float target_speed_left = PID_Update(&pid_position_left, target_position_left, g_current_position_left);
    
//     // --- 内环 (速度环) 执行 ---
//     // 4. 获取当前实际速度
//     float current_speed_left = (float)ticks_now_left; // 注意：这里需要重新获取一次，因为上面的累加已经消耗了上次的值
//                                                           // 一个更优的做法是把 Encoder_Get_Left() 的值先存起来
//                                                           // int32_t ticks_now = Encoder_Get_Left();
//                                                           // g_current_position_left += ticks_now;
//                                                           // ...
//                                                           // float current_speed_left = (float)ticks_now;

//     // 5. 更新速度环 PID，其目标是【外环的输出】
//     int16_t pwm_out_left = (int16_t)PID_Update(&pid_speed_left, target_speed_left, current_speed_left);

//     // 6. 应用到电机
//     Motor_SetSpeed_Left(pwm_out_left);

//     // 7. 打印位置环的调试数据
//     // 格式: 目标位置, 当前位置, 外环输出(目标速度)
//     Serial_Printf("%f,%f,%f\r\n", target_position_left, g_current_position_left, target_speed_left);
		

// #elif (CONTROL_MODE == 3)		

//  // ==================== 左轮串级 PID 控制 ====================
    
//     // --- 外环 (位置环) ---
//     // 1. 获取编码器增量，并累加得到当前位置
 
//     g_current_position_left += ticks_now_left;

//     // 2. 更新位置环 PID，其输出是【内环的目标速度】
//     float target_speed_left = PID_Update(&pid_position_left, g_target_position_left, g_current_position_left);
    
//     // --- 内环 (速度环) ---
//     // 3. 获取当前实际速度 (单位: ticks/10ms)
//     float current_speed_left = (float)ticks_now_left;

//     // 4. 更新速度环 PID，其目标是【外环的输出】
//     int16_t pwm_out_left = (int16_t)PID_Update(&pid_speed_left, target_speed_left, current_speed_left);

//     // 5. 应用到电机
//     Motor_SetSpeed_Left(pwm_out_left);


//     // ==================== 右轮串级 PID 控制 (逻辑相同) ====================
//     // (等你的新驱动板到了之后，取消这里的注释即可)
    
//      g_current_position_right += ticks_now_right;
//      float target_speed_right = PID_Update(&pid_position_right, g_target_position_right, g_current_position_right);
//      float current_speed_right = (float)ticks_now_right;
//      int16_t pwm_out_right = (int16_t)PID_Update(&pid_speed_right, target_speed_right, current_speed_right);
//      Motor_SetSpeed_Right(pwm_out_right);
    
// //   Motor_SetSpeed_Right(0); // 调试期间先停止右轮


//     // ==================== 串口打印调试信息 ====================
//     // 打印位置信息来观察定位效果
//     // 格式：目标位置, 当前位置, 外环输出(目标速度), 内环输出(PWM)
//     Serial_Printf("%f,%f,%f,%d,%f,%f,%f,%d\r\n", 
//                   g_target_position_left, 
//                   g_current_position_left, 
//                   target_speed_left, 
//                   pwm_out_left,
// 									g_target_position_right, 
//                   g_current_position_right, 
//                   target_speed_right, 
//                   pwm_out_right);

// //************************************************************************************

// #elif (CONTROL_MODE == 4)	
//  // ============ 1. 从 ROS 获取目标速度 (运动学逆解) ============
    
    
//     // 将 ROS 的 m/s 转换为 cm/s
//     float target_vx_cmps = linear_velocity_x * 100.0f;
//     // vth (rad/s) 不需要转换
//     float target_vth_radps = angular_velocity_z;

//     // 运动学逆解，计算左右轮的目标速度 (cm/s)
//     float target_speed_right_cmps = target_vx_cmps + (target_vth_radps * WHEEL_SEPARATION_CM / 2.0f);
//     float target_speed_left_cmps  = target_vx_cmps - (target_vth_radps * WHEEL_SEPARATION_CM / 2.0f);

//     // ============ 2. 左右轮 PID 闭环控制 ============
//     // (这里的逻辑与你调试好的串级 PID 类似，但目标来自于上面的计算)
//     // 【此处简化为速度单环，你也可以保留串级PID】
//     // 左轮
//     // ============ 左右轮 PID 闭环控制 ============
    
//     // --- 左轮 ---
    
//     float current_speed_left = Calculate_Speed_CMPS(ticks_now_left);
    
//     // --- 右轮 ---
    
//     float current_speed_right = Calculate_Speed_CMPS(ticks_now_right);

//     // ============ 【关键】增加速度死区判断 ============
//     // 如果速度的绝对值非常小 (例如小于 0.1 cm/s)，就强制认为是 0
//     // 这个阈值需要根据你的实际情况微调
//     if (fabs(current_speed_left) < 0.1f) {
//         current_speed_left = 0.0f;
//     }
//     if (fabs(current_speed_right) < 0.1f) {
//         current_speed_right = 0.0f;
//     }
    
//     // 【用处理后的速度进行 PID 计算】
//     int16_t pwm_left = (int16_t)PID_Update(&pid_speed_left, target_speed_left_cmps, current_speed_left);
//     Motor_SetSpeed_Left(pwm_left);
    
//     int16_t pwm_right = (int16_t)PID_Update(&pid_speed_right, target_speed_right_cmps, current_speed_right);
//     Motor_SetSpeed_Right(pwm_right);

//     // ============ 3. 更新并计算里程计 (运动学正解) ============
//     // 使用【实际测量】的速度来计算里程计，而不是目标速度
    
//     // a. 计算机器人的实际线速度和角速度
//     g_vel_vx_cmps = (current_speed_right + current_speed_left) / 2.0f;
//     g_vel_vth_radps = (current_speed_right - current_speed_left) / WHEEL_SEPARATION_CM;
    
//     // b. 对位姿进行积分
//     float dt = CONTROL_PERIOD_S;
//     g_pos_x_cm += g_vel_vx_cmps * cos(g_pos_th_rad) * dt;
//     g_pos_y_cm += g_vel_vx_cmps * sin(g_pos_th_rad) * dt;
//     g_pos_th_rad += g_vel_vth_radps * dt;

//     // (可选) 角度规范化到 -PI 到 PI
//     // if (g_pos_th_rad > 3.14159f) g_pos_th_rad -= 2 * 3.14159f;
//     // if (g_pos_th_rad < -3.14159f) g_pos_th_rad += 2 * 3.14159f;

// #endif


// }
