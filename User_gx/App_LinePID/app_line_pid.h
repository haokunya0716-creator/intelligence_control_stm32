//
// Created by gaoxu on 2026/10/17.
//

#ifndef INTELLIGENT_RESCUE_APP_LINE_PID_H
#define INTELLIGENT_RESCUE_APP_LINE_PID_H

#include "pid.h"
#include <stdint.h>

// 任务视觉参数统一放在App_Task/app_task_config.h，避免两处修改不一致。
#include "app_task_config.h"

// @简介：初始化视觉对准 PID，SP 固定为屏幕中心 LINE_PID_AIM_X。
void App_LinePID_Init(void);

// @简介：换目标或首次对准结束时，复位历史误差、积分项和死区状态。
void App_LinePID_Reset(void);

// @简介：根据目标中心 x 计算差速修正量，已按死区和符号处理，单位 cm/s。
float App_LinePID_Compute(float object_cx);

// @简介：接近停车门限时使用更严格的双门限死区，差速输出单位cm/s。
float App_LinePID_ComputeNear(float object_cx);

// @简介：目标中心是否进入内死区，用于结束首次原地对准。
uint8_t App_LinePID_InDeadband(float object_cx);

#endif //INTELLIGENT_RESCUE_APP_LINE_PID_H
