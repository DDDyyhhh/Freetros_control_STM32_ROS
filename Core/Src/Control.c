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
#define CONTROL_MODE 0  // <<< 在这里切换模式：1调试速度环 或 2调试位置环 或 3双轮串级pid 或4 与ROS通信控制
// ==========================================================
// ======================= 全局变量及常量定义 =======================

// --- PID 实例 ---
PID_t pid_speed[MOTOR_NUM]; // 为4个电机创建PID实例数组

// --- 物理参数 (需要为麦克纳姆轮重新标定!) ---
// 这些参数对运动的准确性至关重要
const float WHEEL_DIAMETER_CM = 6.0f;       // 麦轮直径 (cm)
const float WHEEL_BASE_CM = 20.0f;          // 左右轮距 (cm)
const float TRACK_WIDTH_CM = 22.0f;         // 前后轮距 (cm)
const float ENCODER_TICKS_PER_REV = 4454.4f;   // 编码器每转一圈的脉冲数 (减速后)
const float CONTROL_PERIOD_S = 0.01f;       // 控制周期 (秒, 10ms)

// --- 根据物理参数计算出的转换系数 ---
// 每 tick 对应的距离(cm) = (轮子周长) / (每转tick数)
const float CM_PER_TICK = (3.1415926f * WHEEL_DIAMETER_CM) / ENCODER_TICKS_PER_REV;

// --- 视觉伺服控制参数 (需要调试) ---
const int16_t IMAGE_CENTER_X = 160;     // 假设图像中心X坐标
const int16_t TARGET_Y_COORD = 200;     // 希望垃圾最终停在图像的这个Y坐标上 (表示距离合适)
const float KP_ANGULAR = 0.05f;         // 视觉旋转控制P增益
const float KP_LINEAR = 0.1f;           // 视觉前进控制P增益

// --- 共享的调试数据 (由互斥锁保护) ---
float g_target_speeds_cmps[MOTOR_NUM] = {0};
float g_current_speeds_cmps[MOTOR_NUM] = {0};
int16_t g_pwm_out[MOTOR_NUM] = {0};

uint8_t current_motor;
int16_t TEST_PWM = 30; // 使用一个较小的PWM值进行测试

uint8_t motor_to_read;
int32_t encoder_val; // 使用 int32_t 以匹配 Encoder_Get_Increment 的返回值

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
		#if (CONTROL_MODE == 1) // 如果是模式1：调试速度环
    // === 内环-速度环 PID 参数 ===
    // 从这里开始调试你的 Kp, Ki, Kd
    PID_Init(&pid_speed[0], 20.0f, 10.0f, 0.5f, MAX_PWM, -MAX_PWM);
    
    // 外环不起作用，参数设为 0
    PID_Init(&pid_position_left, 0.0f, 0.0f, 0.0f, 80.0f, -80.0f);

		#elif (CONTROL_MODE == 2) // 如果是模式2：调试位置环
				// === 内环-速度环 PID 参数 ===
				// 【关键】在这里填入你最终调试好的速度环完美参数！
				// 假设你最终调好的值是 Kp=2.5, Ki=0.8, Kd=0.1
				PID_Init(&pid_speed_left, 0.87f, 0.0f, 0.0f, 100.0f, -100.0f);

				// === 外环-位置环 PID 参数 ===
				// 现在开始调试位置环的 Kp, Ki, Kd
				PID_Init(&pid_position_left, 5.0f, 0.0f, 0.0f, 80.0f, -80.0f);
		
		#elif (CONTROL_MODE == 3 ) // 如果是模式3：双轮串级pid
		
				//******************双环pid***********************
// === 1. 初始化【内环-速度环】PID 参数 ===
    // 使用你已经调试好的完美参数！
    // 假设你最终调好的值是 Kp=0.2, Ki=0.1 (这只是示例)
    PID_Init(&pid_speed_left, 0.87f, 0.0f, 0.0f, 100.0f, -100.0f);
    PID_Init(&pid_speed_right, 0.87f, 0.0f, 0.0f, 100.0f, -100.0f); // 右轮也用相同参数
		
		
		// === 2. 初始化【外环-位置环】PID 参数 ===
    // 位置环通常只需要 P 控制，或者一个很小的 D 控制。Ki 通常为 0。
    // P 控制器：Kp 决定了小车以多快的速度去接近目标位置。
    // OutMax/OutMin 限制了位置环输出的最大速度。
    PID_Init(&pid_position_left, 5.0f, 0.0f, 0.0f, 80.0f, -80.0f); // Kp从0.1开始，最大速度限制在80 ticks/10ms
    PID_Init(&pid_position_right, 5.0f, 0.0f, 0.0f, 80.0f, -80.0f);
		
		#elif (CONTROL_MODE ==   4)
		const int16_t MAX_PWM = 99; // 与你的Motor_SetSpeed中的MAX_PWM保持一致

    // 使用循环为4个电机初始化PID控制器
    for (int i = 0; i < MOTOR_NUM; i++) {
        // 初始PID参数，需要重新调试
        // Kp, Ki, Kd 的单位现在是针对 (cm/s) 的误差
        // OutMax/OutMin 的单位是 PWM 值
        PID_Init(&pid_speed[i], 20.0f, 10.0f, 0.5f, MAX_PWM, -MAX_PWM);
    }
		
		#endif

    
    
		
		//*******************************************************************

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

// ======================= 工具函数 =======================

/**
  * @brief 根据编码器脉冲数计算轮子线速度 (cm/s)
  */
static inline float TicksToSpeedCMPS(int16_t ticks)
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
    
    osDelay(100); // 每100ms打印一次

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
        Serial_Printf("%.2f,%.2f,%d\r\n", local_target, local_current, local_pwm);
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