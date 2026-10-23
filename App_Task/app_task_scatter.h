//
// Created by gaoxu on 2026/10/20.
//

#ifndef INTELLIGENT_RESCUE_APP_TASK_SCATTER_H // 检查头文件保护宏，防止重复包含时反复展开声明。
#define INTELLIGENT_RESCUE_APP_TASK_SCATTER_H // 定义头文件保护宏，后续重复包含时跳过本文件。

#include "app_task_move.h" // 引入运动动作及统一结果类型。

// @简介：视觉对准并走到物块面前后调用一次；开始额外前推，不重新选择目标。
void App_Task_Scatter_Start(void); // 声明接口：视觉对准并走到物块面前后调用一次；开始额外前推，不重新选择目标。

// @简介：非阻塞执行“编码器额外前推→限时原地旋转→停车”，每轮主循环调用。
// @返回：DONE仅表示动作结束，上层必须换view_id重检；异常返回FAILED且停车。
App_Task_Result App_Task_Scatter_Proc(void); // 声明接口：非阻塞执行“编码器额外前推→限时原地旋转→停车”，每轮主循环调用。

#endif // 结束本头文件的重复包含保护。
