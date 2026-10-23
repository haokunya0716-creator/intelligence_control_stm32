//
// Created by gaoxu on 2026/10/17.
//

#include "app_line_pid.h"

static PID_TypeDef line_pid; // 视觉对准 PID，SP 为屏幕中心 x，反馈为目标中心 x
static uint8_t correcting;   // 双门限死区开关，防止目标在死区边界来回摆荡

//
// @简介：初始化视觉对准 PID，输出限幅为单侧差速上限。
//
void App_LinePID_Init(void)
{
    PID_Init(&line_pid, LINE_PID_KP, LINE_PID_KI, LINE_PID_KD);
    PID_ChangeSP(&line_pid, LINE_PID_AIM_X); // 设定值就是屏幕中心
    PID_LimitConfig(&line_pid, LINE_PID_STEER_MAX, -LINE_PID_STEER_MAX);
    correcting = 1;
}

//
// @简介：切换目标或首次对准结束时调用，清掉上一次的误差历史。
//
void App_LinePID_Reset(void)
{
    PID_Reset(&line_pid);
    correcting = 1;
}

//
// @简介：根据目标中心 x 计算差速修正量。
// @参数 object_cx：目标中心 x，按 640x480 归一化后的像素值。
// @返回值：正数表示需要向右修正，负数表示向左，已限幅，单位 cm/s。
// @说明：使用双门限死区，误差进入内门限停止修正，超过外门限才重新修正。
//
static float App_LinePID_ComputeBand(float object_cx, float enter, float exit)
{
    float error = object_cx - LINE_PID_AIM_X;
    float turn;

    if(error <= enter && error >= -enter){
        correcting = 0; // 误差足够小，停止修正
    }else if(error >= exit || error <= -exit){
        correcting = 1; // 越过外门限才重新修正
    }
    if(correcting == 0){
        PID_Reset(&line_pid); // 死区内不积累历史，恢复修正时从当前误差开始
        return 0.0f;
    }
    turn = -PID_Compute(&line_pid, object_cx) * LINE_PID_STEER_SIGN; // PID 是 SP-FB，右侧目标输出为负，取反后正差速右转
    if(turn > LINE_PID_STEER_MAX){
        turn = LINE_PID_STEER_MAX;
    }
    if(turn < -LINE_PID_STEER_MAX){
        turn = -LINE_PID_STEER_MAX;
    }
    return turn;
}

// @简介：远处/首次对准使用10/20像素双门限死区，输出左右轮差速修正量。
float App_LinePID_Compute(float object_cx)
{
    return App_LinePID_ComputeBand(object_cx, LINE_PID_DEAD_ENTER, LINE_PID_DEAD_EXIT);
}

// @简介：近处使用7/8像素双门限死区，提早恢复纠偏；不改变前进状态，不原地调整。
float App_LinePID_ComputeNear(float object_cx)
{
    return App_LinePID_ComputeBand(object_cx, LINE_PID_NEAR_ENTER, LINE_PID_NEAR_EXIT);
}

//
// @简介：目标中心是否在死区内。
// @返回值：1 在死区内，0 不在。
//
uint8_t App_LinePID_InDeadband(float object_cx)
{
    float error = object_cx - LINE_PID_AIM_X;
    return error <= LINE_PID_DEAD_ENTER && error >= -LINE_PID_DEAD_ENTER ? 1U : 0U;
}
