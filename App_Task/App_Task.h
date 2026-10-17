//
// Created by gaoxu on 2026/10/13.
//

#ifndef INTELLIGENT_RESCUE_APP_TASK_H
#define INTELLIGENT_RESCUE_APP_TASK_H

#include "main.h"
#include "rescue_protocol.h"

// 启动直行参数：时间单位 ms，速度单位 cm/s，修改后重新编译即可。
#define APP_TASK_FORWARD_TIME_MS    2000U
#define APP_TASK_FORWARD_SPEED      20.0f

// 转向是底盘原地转向，相机保持朝车头方向。
// 按当前角度环的轮速输出设置：负值右转，正值左转；可按实车修改符号。
// 先向右 90°，再转 180° 到初始车头方向的左侧。
#define APP_TASK_RIGHT_ANGLE        (-90.0f)
#define APP_TASK_LEFT_ANGLE         180.0f

// 转向结束后等待画面稳定，再通知相机采集新视角；单位均为 ms。
#define APP_TASK_SETTLE_TIME_MS     200U
#define APP_TASK_VIEW_RETRY_MS      100U
#define APP_TASK_DETECT_TIMEOUT_MS  2000U
#define APP_TASK_TURN_TIMEOUT_MS    5000U

// 像素参数统一按 640×480 换算，实际相机分辨率可不同。
#define APP_TASK_AIM_X              320.0f // 前框中心对应的画面 x，需标定
#define APP_TASK_CONFIDENCE_MIN     70U
#define APP_TASK_ISOLATE_GAP_X      40.0f // 框间水平/垂直间隔，非厘米距离
#define APP_TASK_ISOLATE_GAP_Y      30.0f
#define APP_TASK_PATH_HALF_WIDTH    70.0f // 接近通道左右预留，按车宽调试
#define APP_TASK_PATH_DEPTH_MARGIN  20.0f // 底边明显更靠下，视为前方遮挡
#define APP_TASK_TRACK_GATE         70.0f // 相邻帧中心移动门限
#define APP_TASK_TRACK_AMBIGUITY    20.0f // 最近/次近平方距离差小于此值平方则放弃

// 双门限死区：进入 10 px 停止修正；超过 20 px 才重新修正。
#define APP_TASK_AIM_DEAD_ENTER     10.0f
#define APP_TASK_AIM_DEAD_EXIT      20.0f
#define APP_TASK_AIM_REALIGN        70.0f // 偏差过大，停车重新原地对准
#define APP_TASK_STEER_KP           0.08f // cm/s/px
#define APP_TASK_STEER_MAX          4.0f  // 单侧差速上限，cm/s
#define APP_TASK_STEER_SIGN         1.0f  // 目标在右时左轮快；实车反向改为 -1
#define APP_TASK_APPROACH_SPEED     8.0f  // 接近速度，cm/s
#define APP_TASK_APPROACH_SLOW      4.0f
#define APP_TASK_SLOW_MARGIN_CM     15.0f
#define APP_TASK_APPROACH_STOP_CM   20.0f // 单独绿色的观察停车距离，尚不接触
#define APP_TASK_SCATTER_STOP_CM    30.0f // 分离动作前的准备距离，待调试

// 固定摄像头俯角，两点标定绿色物块框底边和到前框的距离。
// y 按 480 高度填写；默认 0 表示未标定，对准后停车，不盲目前进。
// NEAR_Y > FAR_Y；距离只在两点覆盖范围内估算。
#ifndef APP_TASK_RANGE_NEAR_Y
#define APP_TASK_RANGE_NEAR_Y       0.0f
#endif
#ifndef APP_TASK_RANGE_FAR_Y
#define APP_TASK_RANGE_FAR_Y        0.0f
#endif
#define APP_TASK_RANGE_NEAR_CM      20.0f
#define APP_TASK_RANGE_FAR_CM       60.0f
#define APP_TASK_CAMERA_FRONT_CM    0.0f  // 相机到前框的前后间距；相机在后为正

#define APP_TASK_TRACK_MAX_AGE_MS   200U
#define APP_TASK_ALIGN_TIMEOUT_MS   5000U
#define APP_TASK_APPROACH_TIMEOUT_MS 15000U
#define APP_TASK_SCATTER_TIMEOUT_MS 5000U
#define APP_TASK_ROUND_TIME_MS      180000U // 整轮计时，再次按键不重置

typedef enum {
    APP_TASK_SCATTER_WAIT = 0, // 尚未实现/等待，默认保持停车
    APP_TASK_SCATTER_RUNNING,  // 动作进行中
    APP_TASK_SCATTER_DONE,     // 动作结束，丢弃旧锁定并重新观察
    APP_TASK_SCATTER_FAILED
} App_Task_ScatterResult;

#define SERVO1_LIFT  180.0f//前框抬起的角度
#define SERVO1_DROP  90.0f//前框落下的角度，框住小物块
#define SERVO1_SUPPORT  0.0f//前框支撑的角度，脱困用

#define SERVO2_LIFT  75.0f//摄像机抬起的角度（正常寻找物块时使用）
#define SERVO2_DROP  45.0f//摄像机落下的角度（前框已框下，确认物块信息使用）
// @简介：按键启动流程；执行中再次调用不会重复启动。
void App_Task_Start(void);

// @简介：任务状态机，在 while(1) 中持续调用，不使用阻塞延时。
void App_Task_Proc(void);

// 非阻塞物块分离接口；当前仅占位，返回 WAIT，不驱动电机/舵机。
// target 是进入接口前的原始像素框，只供规划，不是持续更新的坐标。
// 后续实现自行读取新图像并运行闭环；返回 DONE 后上层重新采集新视角。
App_Task_ScatterResult App_Task_Scatter_Proc(const RescueObject *target);


void App_Servo_Set();

#endif //INTELLIGENT_RESCUE_APP_TASK_H
