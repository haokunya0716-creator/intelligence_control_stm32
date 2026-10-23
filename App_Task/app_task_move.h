//
// Created by gaoxu on 2026/10/20.
//

#ifndef INTELLIGENT_RESCUE_APP_TASK_MOVE_H // 检查头文件保护宏，防止重复包含时反复展开声明。
#define INTELLIGENT_RESCUE_APP_TASK_MOVE_H // 定义头文件保护宏，后续重复包含时跳过本文件。

#include "main.h" // 引入HAL基础定义及公共类型。
#include "app_task_config.h" // 引入任务集中配置宏。

typedef enum { // 定义各动作模块统一使用的步骤结果枚举。
    APP_STEP_WAIT = 0, // 等待新画面或机械稳定，保持当前安全状态
    APP_STEP_RUNNING,  // 动作进行中，主循环继续调用
    APP_STEP_DONE,     // 本步骤完成，由任务层决定下一步
    APP_STEP_FAILED,   // 异常结束，运动模块已停车
    APP_STEP_RESEARCH  // 安全区跟随条件失配，已立即转入旋转搜索，由上层切换状态。
} App_Task_Result; // 将以上步骤结果命名为App_Task_Result，供任务状态机判断。

// @简介：撤销位置/角度控制并停止双轮，不修改任务状态和载物标志。
void App_Task_Move_Stop(void); // 声明接口：撤销位置/角度控制并停止双轮，不修改任务状态和载物标志。

// @简介：设置左右目标轮速并执行速度闭环；单位cm/s，正值前进。
void App_Task_Move_Speed(float left, float right); // 声明接口：设置左右目标轮速并执行速度闭环；单位cm/s，正值前进。

// @简介：帧间继续执行速度内环，不重新修改轮速目标或运行视觉PID。
void App_Task_Move_Proc(void); // 声明接口：帧间继续执行速度内环，不重新修改轮速目标或运行视觉PID。

// @简介：开始相对角度转向；angle单位度，逆时针为正。
void App_Task_Turn_Start(float angle); // 声明接口：开始相对角度转向；angle单位度，逆时针为正。

// @简介：非阻塞执行角度闭环；完成/超时均停车，返回步骤结果。
App_Task_Result App_Task_Turn_Proc(void); // 声明接口：非阻塞执行角度闭环；完成/超时均停车，返回步骤结果。

// @简介：记录两轮当前位置并开始直线移动；distance为有符号厘米，speed填正cm/s。
// @说明：双轮分别累计行程，不清零编码器；再次调用会开始一段新的行程。
void App_Task_Move_Start(float distance, float speed); // 声明接口：记录两轮当前位置并开始直线移动；distance为有符号厘米，speed填正cm/s。

// @简介：执行编码器行程和速度闭环；两轮都达到才完成，反向/偏差过大/超时则失败。
App_Task_Result App_Task_Move_Distance(void); // 声明接口：执行编码器行程和速度闭环；两轮都达到才完成，反向/偏差过大/超时则失败。

// @简介：设置一号前框角度；down=1落框，down=0抬框，机械等待由任务层安排。
void App_Task_Frame_Set(uint8_t down); // 声明接口：设置一号前框角度；down=1落框，down=0抬框，机械等待由任务层安排。

// @简介：设置二号摄像头角度；down=1低头复检，down=0正常观察，同时更新舵机结构体。
void App_Task_Camera_Set(uint8_t down); // 声明接口：设置二号摄像头角度；down=1低头复检，down=0正常观察，同时更新舵机结构体。

// @简介：退离物资区后的新循环接口，目前留空；不能把抬框直接当成首运成功。
void App_Task_NextCycle_Proc(void); // 声明接口：退离物资区后的新循环接口，目前留空；不能把抬框直接当成首运成功。

#endif // 结束本头文件的重复包含保护。
