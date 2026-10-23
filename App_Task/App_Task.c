//
// Created by gaoxu on 2026/10/13.
//

#include "App_Task.h" // 引入任务启动和状态机接口。
#include "app_task_move.h" // 引入运动动作及统一结果类型。
#include "app_task_vision.h" // 引入视觉选择、跟随和复检接口。
#include "app_task_scatter.h" // 引入撞散前推及旋转接口。
#include "app_usart.h" // 引入串口1格式化调试输出接口。

typedef enum { // 定义任务状态枚举，状态机按下列步骤组织运行。
    TASK_IDLE = 0,    // 停车等待按键
    TASK_FORWARD,     // 启动直行
    TASK_SEARCH_SPIN = 4, // 启动直行后持续顺时针旋转找绿色，沿用原4号搜索日志。
    TASK_RESELECT = 6, // 撞散结束后重新观察，保留原状态编号。
    TASK_FOLLOW,      // 首次对准+视觉PID接近，细节由视觉模块执行
    TASK_SCATTER,     // 额外前推+限时原地旋转
    TASK_FRAME_DROP, // 等待一号前框落下后直接复检，摄像头保持正常角度。
    TASK_VERIFY = 11, // 三张新图中至少一张单绿色合格；保留原日志编号，删除原10号摄像头低头等待。
    TASK_ZONE_SEARCH = 14, // 复检通过直接原地寻找本队物资放置区，保留原状态编号。
    TASK_ZONE_FOLLOW, // 对准并接近锁定的本队物资放置区。
    TASK_FRAME_LIFT, // 到达物资区停车门限后等待前框抬起。
    TASK_RELEASE_BACK, // 抬框后按编码器行程退离物资区。
    TASK_NEXT_CYCLE   // 新循环预留，不代表已确认首运得分
} App_Task_State; // 将以上状态枚举命名为App_Task_State。

static App_Task_State task_state = TASK_IDLE; // 当前任务状态，程序启动时默认为停车等待。
static uint32_t task_time;    // 当前任务步骤起点，ms；子动作自己维护内部计时
static uint32_t round_time;   // 第一次按键起点，后续重试不延长整轮时间
static uint8_t round_started; // 整轮是否已开始计时，静态初值为0；后续重试不清零。
static uint8_t cargo_pending; // 落框后即使失败也保留，禁止按键盲目重启

// @简介：切换任务步骤并记录开始时刻；只用于状态流转，不控制底层硬件。
static void App_Task_Change(App_Task_State state) // 函数入口：切换任务步骤并记录开始时刻；只用于状态流转，不控制底层硬件。
{ // 开始App_Task_Change函数体。
    task_state = state; // 保存新任务状态，下次处理时执行对应分支。
    task_time = HAL_GetTick(); // 记录该步骤开始的毫秒时刻，供等待及超时判断。
    if(APP_TASK_DEBUG_ENABLE){ // 仅在开启任务调试宏时输出本次调试信息。
        App_USART1_Printf("[TASK] state=%u\r\n", (unsigned int)state); // 通过串口1打印新状态编号，便于对应枚举观察流程。
    } // 结束条件分支。
} // 结束App_Task_Change函数体。

// @简介：结束本次任务并停车；不清除落框标志，异常载物需人工处理后复位。
static void App_Task_Stop(const char *reason) // 先停车，再打印退出原因及原状态，最后回到空闲。
{
    uint32_t now_ms = HAL_GetTick(); // 保存打印前的时间，避免串口耗时改变本次现场记录。
    App_Task_Move_Stop(); // 先撤销运动，不让失败打印延后停车。
    if(APP_TASK_DEBUG_ENABLE){
        App_USART1_Printf("[TASK] stop=%s state=%u dt=%lu round=%lu\r\n", reason, (unsigned int)task_state,
                         (unsigned long)(now_ms - task_time),
                         (unsigned long)(round_started ? now_ms - round_time : 0U));
    }
    App_Task_Change(TASK_IDLE); // 保留载物锁及整轮起点。
}

// @简介：停车后开始正常俯角的新视角检测；旧图像不能用于撞散后重选或搜索物资区。
static void App_Task_BeginDetect(App_Task_State state) // 函数入口：停车后开始正常俯角的新视角检测；旧图像不能用于撞散后重选或搜索物资区。
{ // 开始App_Task_BeginDetect函数体。
    App_Task_Vision_Begin(SERVO2_LIFT, state == TASK_RESELECT ? APP_TASK_SETTLE_TIME_MS : 0U); // 仅撞散重选等待画面稳定；启动旋转和物资区搜索立即发新视角。
    App_Task_Change(state); // 切换到指定检测状态，并记录本步骤起点。
} // 结束App_Task_BeginDetect函数体。

// @简介：空闲且未落框时选择队伍并启动；整轮三分钟计时只在第一次启动时开始。
void App_Task_Start(uint8_t team) // 函数入口：空闲且未落框时选择队伍并启动；整轮三分钟计时只在第一次启动时开始。
{ // 开始App_Task_Start函数体。
    uint32_t now_ms = HAL_GetTick(); // 读取当前毫秒计时，作为本次处理的时间基准。
    const char *reject = 0; // 只记录原有拒绝条件，不新增启动限制。
    if(task_state != TASK_IDLE){ reject = "busy"; }
    else if(cargo_pending != 0){ reject = "cargo_pending"; }
    else if(team != APP_TASK_TEAM_RED && team != APP_TASK_TEAM_BLUE){ reject = "invalid_team"; }
    else if(round_started != 0 && (uint32_t)(now_ms - round_time) >= APP_TASK_ROUND_TIME_MS){ reject = "round_timeout"; }
    if(reject != 0){
        if(APP_TASK_DEBUG_ENABLE){
            App_USART1_Printf("[TASK] reject=%s state=%u team=%u cargo=%u\r\n",
                             reject, (unsigned int)task_state, (unsigned int)team, (unsigned int)cargo_pending);
        }
        return;
    }
    if(round_started == 0){ // 仅第一次接受启动时建立整轮计时起点。
        round_started = 1; // 标记整轮已启动，后续重试不再重置起点。
        round_time = now_ms; // 保存本轮首次启动的毫秒时间。
    } // 结束条件分支。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    App_Task_Vision_Init(team); // 锁存队伍对应的物资区类型，复位视觉PID及用途标志。
    App_Task_Frame_Set(0); // 命令前框抬起，搜索或退离时保持抬起。
    App_Task_Camera_Set(0); // 命令摄像头恢复正常观察角度。
    App_Task_Move_Speed(APP_TASK_FORWARD_SPEED, APP_TASK_FORWARD_SPEED); // 左右轮均下发启动轮速，正值前进，使用速度闭环。
    App_Task_Change(TASK_FORWARD); // 进入启动直行状态，开始计算该段直行持续时间。
} // 结束App_Task_Start函数体。

// @简介：非阻塞执行小车任务；只写步骤、条件与状态切换，具体动作调用封装函数。
// @说明：任何阶段整轮到时均停车；落框后失败仍保留载物锁，不自动解锁后续救援。
void App_Task_Proc(void) // 函数入口：非阻塞执行小车任务；只写步骤、条件与状态切换，具体动作调用封装函数。
{ // 开始App_Task_Proc函数体。
    App_Task_Result result; // 保存子动作的运行、等待、完成或失败结果。
    uint32_t elapsed = (uint32_t)(HAL_GetTick() - task_time); // 用无符号时间差计算当前步骤已经持续的毫秒数。
    if(round_started != 0 && (uint32_t)(HAL_GetTick() - round_time) >= APP_TASK_ROUND_TIME_MS){ // 整轮到达运行时限时，优先停止当前任务。
        if(task_state != TASK_IDLE){ App_Task_Stop("round_timeout"); } // 若仍有任务在运行则停车并回到空闲状态。
        return; // 结束本次调用，不再执行后续语句。
    } // 结束条件分支。
    switch(task_state){ // 根据当前状态执行一个非阻塞步骤。
    case TASK_FORWARD: // 处理启动后的定时直行。
        if(elapsed >= APP_TASK_FORWARD_TIME_MS){ // 直行时间达到宏定义门限后开始旋转搜索。
            App_Task_BeginDetect(TASK_SEARCH_SPIN); // 立即建立新视角，进入持续顺时针搜索，不再先右转再换向。
            App_Task_Move_Speed(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED); // 与寻找本队物资区共用轮速，左正右负顺时针旋转。
        }else { // 前述条件不满足时执行此分支。
            App_Task_Move_Proc(); // 维持当前目标轮速，继续执行左右轮速度闭环。
        } // 结束其它情况分支。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_SEARCH_SPIN: // 持续顺时针旋转，边转边用当前视角新图选择绿色优先的目标。
        if(App_Task_Vision_Select() > 0){ // 成功锁定合格目标后停止搜索旋转，交给原跟随流程。
            App_Task_Move_Stop(); // 进入首次对准前清除搜索轮速，避免继续沿用旋转命令。
            App_Task_Change(TASK_FOLLOW); // 切换到首次对准和视觉差速接近状态。
        }else { // 尚无合格目标或正在等待新图时继续顺时针旋转。
            App_Task_Move_Speed(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED); // 共用物资区搜索轮速，不受原两侧2秒/三张空图结束条件限制。
        } // 结束其它情况分支。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_RESELECT: { // 处理撞散后的重选，并为局部变量建立作用域。
        int selected; // 保存目标选择结果：正数已锁定、零无候选、负数继续等待。
        selected = elapsed >= APP_TASK_DETECT_TIMEOUT_MS ? 0 : App_Task_Vision_Select(); // 撞散重选超时视为无候选，否则按当前有效新图尝试选择目标。
        if(selected > 0){ // 成功锁定合格目标时进入跟随流程。
            App_Task_Change(TASK_FOLLOW); // 切换到首次对准和视觉差速接近状态。
        }else if(selected == 0){ App_Task_Vision_LogDetails("SELECT"); App_Task_Stop(elapsed >= APP_TASK_DETECT_TIMEOUT_MS ? "reselect_timeout" : "reselect_empty"); } // 撞散重选连续有效新帧无候选或超时仍停车结束。
        break; // 结束当前状态分支，等待下一次任务处理。
    } // 结束检测状态局部作用域。

    case TASK_FOLLOW: // 处理场地物块的首次对准、差速接近和到位复核。
        result = App_Task_Vision_Follow(); // 执行原目标关联、首次对准、差速接近及到位停车复核。
        if(result == APP_STEP_DONE){ // 本步骤完成时进入后续动作分支。
            if(App_Task_Vision_IsMixed()){ // 锁定用途为撞散时进入物资堆分支。
                App_Task_Scatter_Start(); // 检查参数及场景条件后启动额外前推。
                App_Task_Change(TASK_SCATTER); // 转入撞散状态，后续执行前推及限时旋转。
            }else { // 前述条件不满足时执行此分支。
                cargo_pending = 1; // 落框前设置载物锁；此后异常也禁止按键盲目重启。
                App_Task_Frame_Set(1); // 单独绿色走到停车位置后才落框
                App_Task_Change(TASK_FRAME_DROP); // 进入前框落下后的机械等待阶段。
            } // 结束其它情况分支。
        }else if(result == APP_STEP_FAILED){ App_Task_Stop("follow_failed"); } // 子动作失败时统一停车，停止后续任务。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_SCATTER: // 处理额外前推和原地旋转。
        result = App_Task_Scatter_Proc(); // 非阻塞执行撞散子动作并取得其当前结果。
        if(result == APP_STEP_DONE){ // 本步骤完成时进入后续动作分支。
            App_Task_BeginDetect(TASK_RESELECT); // 动作结束不等于撞散成功，重新选目标
        }else if(result == APP_STEP_FAILED){ App_Task_Stop("scatter_failed"); } // 子动作失败时统一停车，停止后续任务。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_FRAME_DROP: // 等待前框落下稳定后直接复检，摄像头不动作、不额外等待。
        if(elapsed >= APP_TASK_SERVO_WAIT_MS){ // 舵机设定角发出后，等待配置的机械稳定时间。
            App_Task_Vision_Begin(SERVO2_LIFT, 0U); // 立即发送复检新视角编号，排除落框前旧图，不再等待200ms。
            App_Task_Change(TASK_VERIFY); // 进入三张新图的框内单绿色复检状态。
        } // 结束条件分支。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_VERIFY: // 收齐三张有效新图后，至少一张区域内恰好一个可信绿色才通过。
        if(elapsed >= APP_TASK_VERIFY_TIMEOUT_MS){ App_Task_Vision_LogDetails("VERIFY"); App_Task_Stop("verify_timeout"); break; } // 复检超过允许时间则停车并退出当前状态处理。
        result = App_Task_Vision_Verify(); // 按中心坐标执行框内实体数量、类型和置信度复检。
        if(result == APP_STEP_DONE){ // 本步骤完成时进入后续动作分支。
            App_Task_BeginDetect(TASK_ZONE_SEARCH); // 复检通过立即发新视角寻找本队物资区，不再后退或等待200ms。
        }else if(result == APP_STEP_FAILED){ App_Task_Stop("verify_failed"); } // 子动作失败时统一停车，停止后续任务。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_ZONE_SEARCH: // 持续寻找本队物资区，直到找到或整轮到时。
        result = App_Task_Vision_SearchZone(); // 旋转搜索唯一可信的本队物资分区，找到后锁定并停车。
        if(result == APP_STEP_DONE){ // 本步骤完成时进入后续动作分支。
            App_Task_Change(TASK_ZONE_FOLLOW); // 进入物资区首次对准及差速接近状态。
        }else if(result == APP_STEP_FAILED){ App_Task_Stop("zone_search_failed"); } // 子动作失败时统一停车，停止后续任务。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_ZONE_FOLLOW: // 对准并接近已锁定的本队物资分区。
        result = App_Task_Vision_Follow(); // 执行原目标关联、首次对准、差速接近及到位停车复核。
        if(result == APP_STEP_DONE){ // 本步骤完成时进入后续动作分支。
            App_Task_Frame_Set(0); // 到达物资区停车门限后抬框，退离后保持抬起
            App_Task_Change(TASK_FRAME_LIFT); // 进入抬框机械等待阶段，等待完成后才退离。
        }else if(result == APP_STEP_RESEARCH){
            App_Task_Change(TASK_ZONE_SEARCH); // 视觉层本次调用已开始旋转，直接切回搜索，不换视角/不固定等待。
        }else if(result == APP_STEP_FAILED){ App_Task_Stop("zone_follow_failed"); } // 子动作失败时统一停车，停止后续任务。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_FRAME_LIFT: // 等待前框抬起稳定后再后退。
        if(elapsed >= APP_TASK_SERVO_WAIT_MS){ // 舵机设定角发出后，等待配置的机械稳定时间。
            App_Task_Move_Start(-APP_TASK_RELEASE_BACK_CM, APP_TASK_BACK_SPEED); // 按配置速度启动卸下物块后的退离行程，负距离表示后退。
            App_Task_Change(TASK_RELEASE_BACK); // 进入退离物资区状态。
        } // 结束条件分支。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_RELEASE_BACK: // 按编码器行程后退，退离后前框保持抬起。
        result = App_Task_Move_Distance(); // 根据两轮编码器行程执行直线运动并读取动作结果。
        if(result == APP_STEP_DONE){ // 本步骤完成时进入后续动作分支。
            App_Task_Change(TASK_NEXT_CYCLE); // 退离完成后进入预留新循环接口，尚不确认首运成功。
        }else if(result == APP_STEP_FAILED){ App_Task_Stop("release_back_failed"); } // 子动作失败时统一停车，停止后续任务。
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_NEXT_CYCLE: // 调用预留的下一轮搜索入口。
        App_Task_NextCycle_Proc(); // 保持停车，等待后续补充送达确认和下一轮搜索
        break; // 结束当前状态分支，等待下一次任务处理。

    case TASK_IDLE: // 空闲状态与默认分支共用下方退出语句。
    default: // 未知状态不下发新动作，结束本次状态处理。
        break; // 结束当前状态分支，等待下一次任务处理。
    } // 结束任务状态选择。
} // 结束App_Task_Proc函数体。
