//
// Created by gaoxu on 2026/10/20.
//

#include "app_task_move.h" // 引入运动动作及统一结果类型。
#include "app_motor.h" // 引入编码器反馈和位置、角度控制接口。
#include "app_speed.h" // 引入轮速设定及速度PID接口。
#include "app_servo.h" // 引入舵机结构体及角度设置接口。
#include "app_usart.h" // 引入串口1格式化调试输出接口。

static uint32_t move_time; // 当前转向或直线运动的毫秒计时起点。
static float move_start_l; // 直线运动开始时左轮累计位置，单位cm。
static float move_start_r; // 直线运动开始时右轮累计位置，单位cm。
static float move_distance; // 本次直线移动的目标行程绝对值，单位cm。
static float move_speed; // 本次直线运动的轮速幅值，单位cm/s，要求为正数。
static float move_sign; // 直线方向符号，前进为1、后退为-1。

// @简介：返回浮点绝对值，用于编码器行程差判断。
static float App_Task_Move_Abs(float value) // 函数入口：返回浮点绝对值，用于编码器行程差判断。
{ // 开始App_Task_Move_Abs函数体。
    return value < 0.0f ? -value : value; // 负值取反、非负值保持，返回浮点绝对值。
} // 结束App_Task_Move_Abs函数体。

// @简介：统一停车，清掉上一次位置/角度环输出，防止不同控制环同时驱动电机。
void App_Task_Move_Stop(void) // 函数入口：统一停车，清掉上一次位置/角度环输出，防止不同控制环同时驱动电机。
{ // 开始App_Task_Move_Stop函数体。
    App_Position_Angle_Reset(); // 复位位置环及角度环，避免旧控制输出干扰当前运动。
    App_Speed_Set(0.0f, 0.0f); // 将左右轮速度目标同时设为零，由底层速度控制停车。
} // 结束App_Task_Move_Stop函数体。

// @简介：直接给左右速度内环设定值，正值向前；视觉对准和撞散旋转均调用此函数。
void App_Task_Move_Speed(float left, float right) // 函数入口：直接给左右速度内环设定值，正值向前；视觉对准和撞散旋转均调用此函数。
{ // 开始App_Task_Move_Speed函数体。
    App_Speed_Set(left, right); // 写入左右轮目标线速度，正值前进，单位cm/s。
    App_Speed_Pro(); // 执行一次左右轮速度PID闭环。
} // 结束App_Task_Move_Speed函数体。

// @简介：同一图像两次处理之间只执行速度内环，不重复累计视觉误差。
void App_Task_Move_Proc(void) // 函数入口：同一图像两次处理之间只执行速度内环，不重复累计视觉误差。
{ // 开始App_Task_Move_Proc函数体。
    App_Speed_Pro(); // 执行一次左右轮速度PID闭环。
} // 结束App_Task_Move_Proc函数体。

// @简介：停车后开始相对转向，记录超时起点；角度符号沿用陀螺仪逆时针为正。
void App_Task_Turn_Start(float angle) // 函数入口：停车后开始相对转向，记录超时起点；角度符号沿用陀螺仪逆时针为正。
{ // 开始App_Task_Turn_Start函数体。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    Set_Angle_SP(angle); // 按当前航向建立相对转角目标，符号沿用逆时针为正。
    move_time = HAL_GetTick(); // 记录当前转向或直线移动的毫秒起点，用于超时保护。
    if(APP_TASK_DEBUG_ENABLE){ // 仅在开启任务调试宏时输出本次调试信息。
        App_USART1_Printf("[TURN] start yaw=%.1f target=%.1f\r\n", angle_yaw, angle_yaw_ref); // 打印转向开始时的实际航向与目标航向，单位度。
    } // 结束条件分支。
} // 结束App_Task_Turn_Start函数体。

// @简介：执行角度环和其速度内环；完成返回DONE，超时返回FAILED并停车。
App_Task_Result App_Task_Turn_Proc(void) // 函数入口：执行角度环和其速度内环；完成返回DONE，超时返回FAILED并停车。
{ // 开始App_Task_Turn_Proc函数体。
    App_Angle_Pro_Limited(APP_TASK_TURN_TOL_DEG, APP_TASK_TURN_MIN_SPEED); // 使用配置的角度容差和最小轮速执行转向闭环。
    if(motor_angle_done == 1){ // 角度环报告到位时，停车并返回完成。
        App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
        if(APP_TASK_DEBUG_ENABLE){ // 仅在开启任务调试宏时输出本次调试信息。
            App_USART1_Printf("[TURN] done yaw=%.1f target=%.1f\r\n", angle_yaw, angle_yaw_ref); // 打印到位时实际航向与目标航向，便于核对角度误差。
        } // 结束条件分支。
        return APP_STEP_DONE; // 报告本步骤完成，由上层任务决定下一步。
    } // 结束条件分支。
    if((uint32_t)(HAL_GetTick() - move_time) >= APP_TASK_TURN_TIMEOUT_MS){ // 转向超过允许时间仍未到位时终止。
        App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
        if(APP_TASK_DEBUG_ENABLE){ // 仅在开启任务调试宏时输出本次调试信息。
            App_USART1_Printf("[TURN] stop=turn_timeout yaw=%.1f target=%.1f\r\n", angle_yaw, angle_yaw_ref); // 打印超时时实际航向与目标航向，便于排查转向未完成。
        } // 结束条件分支。
        return APP_STEP_FAILED; // 报告本步骤失败，不再批准后续动作。
    } // 结束条件分支。
    return APP_STEP_RUNNING; // 动作仍在进行，主循环需继续调用本函数。
} // 结束App_Task_Turn_Proc函数体。

// @简介：开始有符号距离移动；正距离前推，负距离后退，速度参数始终填正数。
void App_Task_Move_Start(float distance, float speed) // 函数入口：开始有符号距离移动；正距离前推，负距离后退，速度参数始终填正数。
{ // 开始App_Task_Move_Start函数体。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    App_Update_Data(); // 更新编码器累计位置及当前运动反馈。
    move_start_l = position_l; // 记录左轮起始位置，不清零全局累计编码器位置。
    move_start_r = position_r; // 记录右轮起始位置，后续以位置差计算行程。
    move_distance = App_Task_Move_Abs(distance); // 取目标距离绝对值，前进后退共用正行程到位门限。
    move_sign = distance < 0.0f ? -1.0f : 1.0f; // 根据距离正负确定运动方向，负距离后退，非负距离前进。
    move_speed = speed; // 保存本次移动轮速幅值，后续控制和参数校验都使用它。
    move_time = HAL_GetTick(); // 记录当前转向或直线移动的毫秒起点，用于超时保护。
    if(speed > 0.0f && move_distance > 0.0f){ // 只有轮速和目标行程都为正时才立即发出运动命令。
        App_Task_Move_Speed(move_sign * speed, move_sign * speed); // 按方向符号给两轮下发相同目标轮速。
    } // 结束条件分支。
} // 结束App_Task_Move_Start函数体。

// @简介：检查两轮实际行程，先到的一轮目标速度设零，另一轮继续走到设定距离。
// @说明：这是编码器距离终止+速度闭环，不是依靠时间估算距离，也不保证地面不打滑。
App_Task_Result App_Task_Move_Distance(void) // 函数入口：检查两轮实际行程，先到的一轮目标速度设零，另一轮继续走到设定距离。
{ // 开始App_Task_Move_Distance函数体。
    float left; // 保存左轮沿指令方向已经走过的行程，单位cm。
    float right; // 保存右轮沿指令方向已经走过的行程，单位cm。
    App_Update_Data(); // 更新编码器累计位置及当前运动反馈。
    left = (position_l - move_start_l) * move_sign; // 左轮位置差乘方向符号，正常前进和后退均得到正行程。
    right = (position_r - move_start_r) * move_sign; // 右轮位置差乘方向符号，反向移动则得到负行程。
    const char *reason = 0; // 按原条件顺序记录首个失败原因。
    if(move_speed <= 0.0f){ reason = "invalid_speed"; }
    else if((uint32_t)(HAL_GetTick() - move_time) >= APP_TASK_BACK_TIMEOUT_MS){ reason = "move_timeout"; }
    else if(left < -APP_TASK_REVERSE_TOL_CM){ reason = "left_reverse"; }
    else if(right < -APP_TASK_REVERSE_TOL_CM){ reason = "right_reverse"; }
    else if(App_Task_Move_Abs(left - right) > APP_TASK_BACK_SKEW_CM){ reason = "wheel_skew"; }
    if(reason != 0){
        App_Task_Move_Stop(); // 先停车，随后打印触发值；原有失败条件和返回值不变。
        if(APP_TASK_DEBUG_ENABLE){
            App_USART1_Printf("[MOVE] stop=%s dt=%lu dir=%.0f speed=%.1f\r\n", reason,
                             (unsigned long)(HAL_GetTick() - move_time), move_sign, move_speed);
            App_USART1_Printf("[MOVE] left=%.4g right=%.4g goal=%.4g skew=%.4g\r\n",
                             left, right, move_distance, App_Task_Move_Abs(left - right));
        }
        return APP_STEP_FAILED;
    }
    if(left >= move_distance && right >= move_distance){ // 必须左右轮都达到目标行程才确认该段移动完成。
        App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
        return APP_STEP_DONE; // 报告本步骤完成，由上层任务决定下一步。
    } // 结束条件分支。
    App_Task_Move_Speed(left >= move_distance ? 0.0f : move_sign * move_speed, // 左轮已到位时目标速度置零，否则保持原方向轮速。
                        right >= move_distance ? 0.0f : move_sign * move_speed); // 右轮独立判断到位，未到位时继续按配置轮速移动。
    return APP_STEP_RUNNING; // 动作仍在进行，主循环需继续调用本函数。
} // 结束App_Task_Move_Distance函数体。

// @简介：控制前框，沿用已有角度宏；这里只发送设定，不阻塞等待机械到位。
void App_Task_Frame_Set(uint8_t down) // 函数入口：控制前框，沿用已有角度宏；这里只发送设定，不阻塞等待机械到位。
{ // 开始App_Task_Frame_Set函数体。
    App_Servo_SetAngle(&servo_1, down ? SERVO1_DROP : SERVO1_LIFT); // 根据down选择前框落下或抬起角，并同步更新一号舵机结构体。
} // 结束App_Task_Frame_Set函数体。

// @简介：控制摄像头俯角；视角编号和图像有效性由视觉模块管理。
void App_Task_Camera_Set(uint8_t down) // 函数入口：控制摄像头俯角；视角编号和图像有效性由视觉模块管理。
{ // 开始App_Task_Camera_Set函数体。
    App_Servo_SetAngle(&servo_2, down ? SERVO2_DROP : SERVO2_LIFT); // 根据down选择摄像头低头或正常观察角，并同步更新二号舵机结构体。
} // 结束App_Task_Camera_Set函数体。

// @简介：下一轮搜索入口预留，当前停车保持前框抬起，不自动解锁其它救援目标。
void App_Task_NextCycle_Proc(void) // 函数入口：下一轮搜索入口预留，当前停车保持前框抬起，不自动解锁其它救援目标。
{ // 开始App_Task_NextCycle_Proc函数体。
    // 后续先补充“恰好一个绿色已进入本队物资区围栏内”的送达确认，再接新循环。
} // 结束App_Task_NextCycle_Proc函数体。
