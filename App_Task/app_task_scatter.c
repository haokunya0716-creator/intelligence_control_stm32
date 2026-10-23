//
// Created by gaoxu on 2026/10/20.
//

#include "app_task_scatter.h" // 引入撞散前推及旋转接口。
#include "app_task_vision.h" // 引入视觉选择、跟随和复检接口。
#include "app_usart.h" // 失败时打印撞散阶段及具体原因。

typedef enum { // 定义撞散子动作的内部阶段枚举。
    SCATTER_IDLE = 0, // 未启动或已结束撞散，静态状态默认取此值。
    SCATTER_PUSH, // 到物块面前后，再额外向前走指定厘米
    SCATTER_SPIN  // 前推完成后原地顺时针旋转
} App_Task_ScatterState; // 将以上撞散阶段命名为App_Task_ScatterState。

static App_Task_ScatterState scatter_state; // 保存当前撞散阶段，静态初始化为SCATTER_IDLE。
static uint32_t scatter_time; // 整次撞散开始时刻，前推与旋转共用此总超时起点。
static const char *const vision_failure = "vision_check";
static const char *scatter_failure = "not_started"; // 保存未启动或退出的原因，不参与控制。
static uint32_t spin_time; // 原地旋转开始时刻，用于限制旋转持续时间。

// @简介：结束撞散并停车；result区分正常完成和异常，任务层负责重检或结束流程。
static App_Task_Result App_Task_Scatter_End(App_Task_Result result) // 函数入口：结束撞散并停车；result区分正常完成和异常，任务层负责重检或结束流程。
{ // 开始App_Task_Scatter_End函数体。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    scatter_state = SCATTER_IDLE; // 清除撞散运行状态，标记当前未在执行撞散。
    return result; // 将调用者指定的完成或失败结果返回给任务层。
} // 结束App_Task_Scatter_End函数体。

// @简介：开始额外前推，前框须已抬起、摄像头为正常观察角，图像检查不通过不启动。
void App_Task_Scatter_Start(void) // 函数入口：开始额外前推，前框须已抬起、摄像头为正常观察角，图像检查不通过不启动。
{ // 开始App_Task_Scatter_Start函数体。
    scatter_time = HAL_GetTick(); // 记录整次撞散的毫秒起点。
    scatter_state = SCATTER_IDLE; // 清除撞散运行状态，标记当前未在执行撞散。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    scatter_failure = 0; // 清除上一次诊断。
    if(APP_TASK_SCATTER_PUSH_CM <= 0.0f){ scatter_failure = "invalid_push_distance"; }
    else if(APP_TASK_SCATTER_PUSH_SPEED <= 0.0f){ scatter_failure = "invalid_push_speed"; }
    else if(APP_TASK_SCATTER_SPIN_SPEED <= 0.0f){ scatter_failure = "invalid_spin_speed"; }
    else if(APP_TASK_SCATTER_SPIN_MS == 0){ scatter_failure = "invalid_spin_time"; }
    else if(App_Task_Vision_ScatterClear() == 0){ scatter_failure = vision_failure; }
    if(scatter_failure != 0){
        if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[SCATTER] reject=%s\r\n", scatter_failure); }
        if(scatter_failure == vision_failure){ App_Task_Vision_LogDetails("SCATTER"); } // 视觉拒绝时打印具体子条件。
        return; // 保留原未启动状态及返回行为。
    }
    App_Task_Move_Start(APP_TASK_SCATTER_PUSH_CM, APP_TASK_SCATTER_PUSH_SPEED); // 通过编码器行程和速度闭环启动额外前推。
    scatter_state = SCATTER_PUSH; // 标记当前已进入撞散前推阶段。
} // 结束App_Task_Scatter_Start函数体。

// @简介：完成额外前推才开始旋转；两种动作均使用速度闭环，不运行角度环。
// @说明：物块接触后会移动，不再强求原目标跨帧匹配；仍检查图像时效、数量和禁碰实体。
App_Task_Result App_Task_Scatter_Proc(void) // 函数入口：完成额外前推才开始旋转；两种动作均使用速度闭环，不运行角度环。
{ // 开始App_Task_Scatter_Proc函数体。
    App_Task_Result result; // 保存子动作的运行、等待、完成或失败结果。
    uint32_t now_ms = HAL_GetTick(); // 读取当前毫秒计时，作为本次处理的时间基准。
    const char *reason = 0; // 保持原失败条件的检查顺序。
    if(scatter_state == SCATTER_IDLE){ reason = scatter_failure != 0 ? scatter_failure : "not_started"; }
    else if((uint32_t)(now_ms - scatter_time) >= APP_TASK_SCATTER_TIMEOUT_MS){ reason = "scatter_timeout"; }
    else if(App_Task_Vision_ScatterClear() == 0){ reason = vision_failure; }
    if(reason != 0){
        unsigned int phase = (unsigned int)scatter_state; // 先保存失败前阶段。
        result = App_Task_Scatter_End(APP_STEP_FAILED); // 先停车及清理状态，再打印。
        if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[SCATTER] stop=%s phase=%u dt=%lu\r\n",
                                  reason, phase, (unsigned long)(now_ms - scatter_time)); }
        if(reason == vision_failure){ App_Task_Vision_LogDetails("SCATTER"); }
        return result;
    }
    if(scatter_state == SCATTER_PUSH){ // 前推未结束时优先处理编码器距离移动。
        result = App_Task_Move_Distance(); // 根据两轮编码器行程执行直线运动并读取动作结果。
        if(result == APP_STEP_FAILED){
            result = App_Task_Scatter_End(APP_STEP_FAILED); // 编码器细节已由MOVE日志打印。
            if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[SCATTER] stop=push_failed phase=1\r\n"); }
            return result;
        } // 前推失败立即停车结束，不继续旋转。
        if(result != APP_STEP_DONE){ return APP_STEP_RUNNING; } // 前推尚未完成时保持运行，等待下一次轮询。
        spin_time = now_ms; // 前推完成后才记录原地旋转起点。
        scatter_state = SCATTER_SPIN; // 切换到原地旋转阶段，本次调用即可继续下发旋转轮速。
    } // 结束条件分支。
    if((uint32_t)(now_ms - spin_time) >= APP_TASK_SCATTER_SPIN_MS){ // 原地旋转持续时间达到门限后结束。
        return App_Task_Scatter_End(APP_STEP_DONE); // 停车并报告动作完成；是否真正撞散仍需任务层重检。
    } // 结束条件分支。
    App_Task_Move_Speed(APP_TASK_SCATTER_SPIN_SPEED, -APP_TASK_SCATTER_SPIN_SPEED); // 左轮前进、右轮后退，以宏定义轮速顺时针原地旋转。
    return APP_STEP_RUNNING; // 动作仍在进行，主循环需继续调用本函数。
} // 结束App_Task_Scatter_Proc函数体。
