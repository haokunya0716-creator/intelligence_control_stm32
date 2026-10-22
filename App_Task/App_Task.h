//
// Created by gaoxu on 2026/10/13.
//

#ifndef INTELLIGENT_RESCUE_APP_TASK_H // 检查头文件保护宏，防止重复包含时反复展开声明。
#define INTELLIGENT_RESCUE_APP_TASK_H // 定义头文件保护宏，后续重复包含时跳过本文件。

#include "main.h" // 引入HAL基础定义及公共类型。
#include "app_task_config.h" // 引入任务集中配置宏。

// @简介：选择队伍并启动；仅空闲且未落框时接受，运行中忽略切队/重启。
// @参数 team：APP_TASK_TEAM_RED或APP_TASK_TEAM_BLUE；重试不重置本轮3分钟计时。
void App_Task_Start(uint8_t team); // 声明接口：选择队伍并启动；仅空闲且未落框时接受，运行中忽略切队/重启。

// @简介：任务执行状态机，在while(1)中持续调用；只安排步骤，不包含底层动作算法。
void App_Task_Proc(void); // 声明接口：任务执行状态机，在while(1)中持续调用；只安排步骤，不包含底层动作算法。

#endif // 结束本头文件的重复包含保护。
