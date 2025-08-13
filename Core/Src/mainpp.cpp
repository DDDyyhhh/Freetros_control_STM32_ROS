// #include "mainpp.h"
// #include "ros.h"
// #include "geometry_msgs/Twist.h"
// #include "main.h"
// #include "pid.h"
// #include "Motor.h" // 假设你已经有了电机驱动库
// #include "std_msgs/Float32.h" // <<<【新增】包含 Float32 消息头文件
// #include <nav_msgs/Odometry.h>     // <<< 包含 Odometry 消息
// #include <tf/transform_broadcaster.h> // <<< 包含 tf 广播头文件
// #include <tf/tf.h>
// double vel[3];

// // === C/C++ 共享变量 ===
// // extern "C" {
// //   float linear_velocity_x = 0.0f;  // m/s
// //   float angular_velocity_z = 0.0f; // rad/s
// //   // 从 Control.c 获取里程计数据
// //   extern float g_pos_x_cm, g_pos_y_cm, g_pos_th_rad;
// //   extern float g_vel_vx_cmps, g_vel_vth_radps;
// // }

// // // 声明由 C 文件实现的接口函数
// // extern "C" {
// //   void Control_Set_Target_Position_Left(float ticks);
// //   void Control_Set_Target_Position_Right(float ticks); // 为右轮也准备好
// // }

// // extern "C" {
// //   void rosserial_idle_cb(void) {
// //     // 这个函数可以什么都不做，因为 yoneken 的 read() 是轮询式的
// //     // 但保留这个接口，是为了将来可能的优化
// //   }
// // }

// // void cmd_vel_cb(const geometry_msgs::Twist& msg) {
// //   linear_velocity_x = msg.linear.x;
// //   angular_velocity_z = msg.angular.z;
// // }
// // // === 回调函数 ===
// // // 当收到 /target_position_left 话题时，此函数被调用
// // void position_cb_left(const std_msgs::Float32& msg) {
// //   // 调用 C 函数，将 ROS 发来的目标位置设置给控制层
// //   Control_Set_Target_Position_Left(msg.data);
// // }


// void vel_callback(const geometry_msgs::Twist &msg)
// {
//    vel[0] = msg.linear.x;  //double vel[3]������mainpp.h
//    vel[1] = msg.linear.y;
//    vel[2] = msg.angular.z;
// }


// // === ROS 对象定义 ===
// ros::NodeHandle nh;
// ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", cmd_vel_cb);

// // 里程计发布
// nav_msgs::Odometry odom_msg;
// ros::Publisher odom_pub("odom", &odom_msg);

// // TF 广播
// geometry_msgs::TransformStamped odom_trans;
// tf::TransformBroadcaster broadcaster;

// char base_link[] = "/base_link";
// char base_footprint[] = "base_footprint"; // <<<【新增
// char odom[] = "/odom";



// //extern "C" {
// //  void rosserial_rx_cb(void) {
// //    nh.getHardware()->reset_rbuf();
// //  }
// //  
// //  void rosserial_tx_cb(void) {
// //    nh.getHardware()->flush();
// //  }
// //}

// void setup(void)
// {
//   nh.initNode();
//   nh.subscribe(sub_cmd_vel);
//   nh.advertise(odom_pub);
//   broadcaster.init(nh); // 初始化 TF 广播
	
// }
// void loop(void)
// {
//     // 1. 获取最新的时间戳
//     ros::Time current_time = nh.now();
// //		static unsigned long last_odom_publish_time = 0;
// //		
// //		if (HAL_GetTick() - last_odom_publish_time > 20) {
//     // 2. 填充并发布 odom -> base_link 的 TF 变换
//     odom_trans.header.stamp = current_time;
//     odom_trans.header.frame_id = odom;
//     odom_trans.child_frame_id = base_footprint;
//     odom_trans.transform.translation.x = g_pos_x_cm / 100.0;
//     odom_trans.transform.translation.y = g_pos_y_cm / 100.0;
//     odom_trans.transform.translation.z = 0.0;
//     odom_trans.transform.rotation = tf::createQuaternionFromYaw(g_pos_th_rad);
//     broadcaster.sendTransform(odom_trans);

//     // 3. 填充并发布 Odometry 消息
//     odom_msg.header.stamp = current_time;
//     odom_msg.header.frame_id = odom;
//     odom_msg.pose.pose.position.x = g_pos_x_cm / 100.0;
//     odom_msg.pose.pose.position.y = g_pos_y_cm / 100.0;
//     odom_msg.pose.pose.orientation = odom_trans.transform.rotation;
//     odom_msg.child_frame_id = base_link;
//     odom_msg.twist.twist.linear.x = g_vel_vx_cmps / 100.0;
//     odom_msg.twist.twist.angular.z = g_vel_vth_radps;
//     odom_pub.publish(&odom_msg);
//     //}
		
//     // 4. 执行一次 ROS 消息处理
//     nh.spinOnce();
// }


